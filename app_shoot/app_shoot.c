#include "app_shoot.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h" // gimbal限位/速度/加速度宏
//
#include "drvs_djimotor_broadcast.h" // DJI 广播纯协议层
#include "drvlib_motor.h"            // 三级级联 PID 控制核
#include "drv_vofa.h"
//
#include "bsp_sys_status.h"
#include "bsp_freertos.h"

// 变量
// 实例
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(friction_motor);    // 摩擦轮 M3508 (C620)
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(friction_motor_group); // 摩擦轮广播组（一条总线一组）
DRVLIB_MOTOR_INSTANCE_DEF(friction_motor_ctrl);          // 摩擦轮速度环（三级 PID 核的中间环）
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(trigger_motor);     // 拨弹 M3508 (C620)
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(trigger_motor_group);  // 拨弹广播组（CAN_1，独立于摩擦轮组）
DRVLIB_MOTOR_INSTANCE_DEF(trigger_motor_ctrl);           // 拨弹 位置环|速度环（三级 PID 核的外+中环）
// 通信
static cmd2shoot_data_t shoot_cmd2shoot_data; // cmd-shoot
static shoot2cmd_data_t shoot_shoot2cmd_data; // shoot-cmd
// 数据
static float friction_motor_setref = 0;
static float trigger_motor_setref = 0; // 拨弹目标角 (rad)

void AppShootInit(void)
{
    // 注册 CAN 实例并挂入广播组
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&friction_motor));
    DrvsDJIMotorBroadcastConfig_s friction_motor_cfg = {
        .group = &friction_motor_group,
        .can_e = CAN_2,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 7,
        .torque_constant = 1, // 转矩常数 (Nm/A)
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&friction_motor, &friction_motor_cfg));

    // 控制核：速度环（ref = rad/s → PID → 电流）
    DrvlibMotor_Config_s friction_motor_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_SPEED,         // 速度闭环
        .position_mode = DRVLIB_MOTOR_POS_CONTINUOUS, // 不用位置环（连续模式，不引入限幅）
        .angle_limit_max = 0,
        .angle_limit_min = 0,
        .position_span = 6.2831853f, // 单圈 2π（M3508 编码器跨度）
        .position_offset = 0,
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE, // 速度/电流低通
        .lpf_rc = 0.02f,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_DISABLE, // 前馈来源
        .speed_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .angle_feedforward_ptr = NULL, // 前馈指针
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .pid =
            {
                [DRVLIB_MOTOR_STAGE_SPEED] =
                    {
                        .kp = 0.1,                                    // 比例系数
                        .ki = 0,                                      // 积分系数
                        .kd = 0,                                      // 微分系数
                        .integral_limit = 0,                          // 积分限幅阈值 (0 = 禁用)
                        .coef_a = 0,                                  // 变速积分参数 A (0 = 禁用)
                        .coef_b = 0,                                  // 变速积分参数 B
                        .d_lpf_rc = 0,                                // 微分滤波时间常数 RC (0 = 禁用)
                        .out_lpf_rc = 0,                              // 输出滤波时间常数 RC (0 = 禁用)
                        .deadband = 0,                                // 死区范围 (0 = 禁用)
                        .error_normalize_range = 0,                   // 误差归一化范围 (0 = 禁用)
                        .out_max = 0,                                 // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
                        .out_min = 0,                                 // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
                        .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL, // 功能配置掩码（Config 也会自动补）
                    },
            },
        .dt_max = 0.01f, // 单帧 dt 上限 (s)
    };
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&friction_motor_ctrl, &friction_motor_ctrl_cfg));

    DrvlibMotorEnable(&friction_motor_ctrl);

    // 拨弹电机：注册 CAN 实例并挂入广播组
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&trigger_motor));
    DrvsDJIMotorBroadcastConfig_s trigger_motor_cfg = {
        .group = &trigger_motor_group,
        .can_e = CAN_1,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 1,
        .torque_constant = 1, // 转矩常数 (Nm/A)
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&trigger_motor, &trigger_motor_cfg));

    // 控制核：位置环|速度环（ref = rad 目标角 → 位置 PID → 速度 PID → 电流）
    DrvlibMotor_Config_s trigger_motor_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED, // 位置(外)→速度(内)
        .position_mode = DRVLIB_MOTOR_POS_CONTINUOUS,                   // 连续模式，不引入限幅
        .angle_limit_max = 0,
        .angle_limit_min = 0,
        .position_span = 6.2831853f, // 单圈 2π（M3508 编码器跨度）
        .position_offset = 0,
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE, // 速度/电流低通
        .lpf_rc = 0.02f,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_DISABLE, // 前馈来源
        .speed_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .angle_feedforward_ptr = NULL, // 前馈指针
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .pid =
            {
                [DRVLIB_MOTOR_STAGE_ANGLE] =
                    {
                        .kp = 0.1,                                    // 比例系数（占位，实机整定）
                        .ki = 0,                                      // 积分系数
                        .kd = 0,                                      // 微分系数
                        .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL, // 功能配置掩码（Config 也会自动补）
                    },
                [DRVLIB_MOTOR_STAGE_SPEED] =
                    {
                        .kp = 0.1,                                    // 比例系数（占位，实机整定）
                        .ki = 0,                                      // 积分系数
                        .kd = 0,                                      // 微分系数
                        .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL, // 功能配置掩码（Config 也会自动补）
                    },
            },
        .dt_max = 0.01f, // 单帧 dt 上限 (s)
    };
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&trigger_motor_ctrl, &trigger_motor_ctrl_cfg));

    DrvlibMotorEnable(&trigger_motor_ctrl);
}

ITCM_RAM void AppShootRun(float dt, uint64_t time_stamp)
{
    (void)time_stamp;
    // 1. 接收
    xQueueReceive(cmd2shoot_queue_handle, &shoot_cmd2shoot_data, 0);

    // 2. 控制
    if (shoot_cmd2shoot_data.fire_or_not == 1)
    {
        friction_motor_setref = 100;
        trigger_motor_setref = 1.0f; // 拨弹目标角 (rad)：占位，实机标定
    }
    else
    {
        friction_motor_setref = 0;
        trigger_motor_setref = 0; // 拨弹回零位
    }

    // 取一帧反馈 → 控制核 → 下发（app 当胶水；方向由 app 加符号，反馈侧取反）
    DrvsDJIMotorBroadcastData_s m = DrvsDJIMotorBroadcastGetData(&friction_motor);
    DrvlibMotorFeedback_s fb = {
        .position = -m.position, // 反馈方向取反
        .speed = -m.speed,       // 反馈方向取反
        .current = m.current,    // 电流不取反
        .timestamp_us = m.timestamp_us,
    };
    DrvsDJIMotorBroadcastSetRef(&friction_motor,
                                -DrvlibMotorSetRef(&friction_motor_ctrl, friction_motor_setref, &fb, dt * 0.001f));
    DrvsDJIMotorBroadcastGroupSend(&friction_motor_group);

    // 拨弹：位置环(外)→速度环(内)（方向待实机确认，暂不加符号）
    DrvsDJIMotorBroadcastData_s t = DrvsDJIMotorBroadcastGetData(&trigger_motor);
    DrvlibMotorFeedback_s trigger_fb = {
        .position = t.position,
        .speed = t.speed,
        .current = t.current,
        .timestamp_us = t.timestamp_us,
    };
    DrvsDJIMotorBroadcastSetRef(&trigger_motor,
                                DrvlibMotorSetRef(&trigger_motor_ctrl, trigger_motor_setref, &trigger_fb, dt * 0.001f));
    DrvsDJIMotorBroadcastGroupSend(&trigger_motor_group);

    // 调试
    // VofaSetChannel(1, friction_motor_ctrl.data.speed);
    // VofaSetChannel(2, friction_motor_setref);
    // VofaSetChannel(3, friction_motor_ctrl.data.current);

    // 3. 发送
    xQueueOverwrite(shoot2cmd_queue_handle, &shoot_shoot2cmd_data); // 通过队列
}
