#include "app_shoot.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h" // gimbal限位/速度/加速度宏
//
#include "drv_motor_base.h"
#include "drv_djimotor.h"
#include "drv_vofa.h"
//
#include "bsp_assert.h"
#include "bsp_freertos.h"

// 变量
// 实例
DJIMOTOR_INSTANCE_DEF(friction_motor); // 摩擦轮 M3508 (C620)
// 通信
static cmd2shoot_data_t shoot_cmd2shoot_data; // cmd-shoot
static shoot2cmd_data_t shoot_shoot2cmd_data; // shoot-cmd
// 数据
static float friction_motor_setref = 0;

void AppShootInit(void)
{
    // 注册 CAN 实例
    BSP_ASSERT_APP_CALL(DJIMotorRegister(&friction_motor));
    DJIMotor_Config_s friction_motor_cfg = {
        .can_e = CAN_2,
        .model = DJI_MODEL_M3508,
        .motor_id = 7,
        .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
        .speed_lpf_rc = 0.02f,
        .position_offset = 0,
        .torque_constant = 1,
        .controller_setting = {
            .loop_type = MOTOR_LOOP_SPEED,                 // 开环：ref 直接作力矩输出，便于方向/转矩常数标定
            .feedback_direction = MOTOR_DIRECTION_REVERSE, // 反馈方向
            .motor_direction = MOTOR_DIRECTION_REVERSE,    // 输出方向
            .position_mode = MOTOR_POSITION_CONTINUOUS,    // 不用位置环（连续模式，不引入限幅）
            .angle_limit_max = 0,
            .angle_limit_min = 0,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针
        },
        .pid_angle_setting = {},
        .pid_speed_setting = {
            .kp = 0.1,                                    // 比例系数
            .ki = 0,                                      // 积分系数
            .kd = 0,                                      // 微分系数
            .integral_limit = 0,                          // 积分限幅阈值 (0 = 禁用)
            .coef_a = 0,                                  // 变速积分参数 A (0 = 禁用)
            .coef_b = 0,                                  // 变速积分参数 B
            .d_lpf_rc = 0,                                // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0,                              // 输出滤波时间常数 RC (0 = 禁用)
            .deadband = 0,                                // 死区范围 (0 = 禁用)
            .error_normalize_range = 0,                   // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
            .out_max = 0,                                 // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .out_min = 0,                                 // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL, // 功能配置掩码

            // PID_ENABLE_INTEGRAL_LIMIT = 0x01,       // 启用积分限幅
            // PID_ENABLE_DERIVATIVE_ON_MEAS = 0x02,   // 启用微分先行
            // PID_ENABLE_CHANGING_INTEGRATION = 0x08, // 启用变速积分
            // PID_ENABLE_PROPORTIONAL_ON_MEAS = 0x10, // 启用比例先行
            // PID_ENABLE_DERIVATIVE_FILTER = 0x20,    // 启用微分滤波
            // PID_ENABLE_OUTPUT_FILTER = 0x40,        // 启用输出滤波
            // PID_ENABLE_OUTPUT_LIMIT = 0x80,         // 启用输出限幅
            // PID_ENABLE_DEADBAND = 0x100,            // 启用死区控制
            // PID_ENABLE_ERROR_NORMALIZE = 0x200,     // 启用误差归一化
        },
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DJIMotorConfig(&friction_motor, &friction_motor_cfg));

    MotorEnable(&(friction_motor.base));
}

ITCM_RAM void AppShootRun(void)
{
    // 1. 接收
    xQueueReceive(cmd2shoot_queue_handle, &shoot_cmd2shoot_data, 0);

    // 2. 控制
    if (shoot_cmd2shoot_data.fire_or_not == 1)
    {
        friction_motor_setref = 100;
    }
    else
    {
        friction_motor_setref = 0;
    }
    MotorSetRef(&(friction_motor.base), friction_motor_setref);
    MotorSend(&(friction_motor.base));
    // 调试
    VofaSetChannel(1, friction_motor.base.data_all.data.speed);
    VofaSetChannel(2, friction_motor_setref);
    VofaSetChannel(3, friction_motor.base.data_all.data.torque);

    // 3. 发送
    xQueueOverwrite(shoot2cmd_queue_handle, &shoot_shoot2cmd_data); // 通过队列
}
