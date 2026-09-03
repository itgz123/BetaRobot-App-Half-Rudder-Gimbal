#include "app_shoot.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h" // gimbal限位/速度/加速度宏
//
#include "drv_motor_base.h"
#include "drv_djimotor.h"
//
#include "bsp_assert.h"

// 实例
DJIMOTOR_INSTANCE_DEF(friction_motor); // 摩擦轮 M3508 (C620)

// 数据
static float friction_motor_setref = 1; // 开环力矩设定 (Nm)；标定时临时修改该值即可输出恒定力矩

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
        .pid_angle_setting = {}, // 开环不参与 PID，预留
        .pid_speed_setting = {
            .kp = 0,                    // 比例系数
            .ki = 0,                    // 积分系数
            .kd = 0,                    // 微分系数
            .integral_limit = 0,        // 积分限幅阈值 (0 = 禁用)
            .coef_a = 0,                // 变速积分参数 A (0 = 禁用)
            .coef_b = 0,                // 变速积分参数 B
            .d_lpf_rc = 0,              // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0,            // 输出滤波时间常数 RC (0 = 禁用)
            .deadband = 0,              // 死区范围 (0 = 禁用)
            .error_normalize_range = 0, // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
            .out_max = 0,               // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .out_min = 0,               // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .config_mask = 0,           // 功能配置掩码
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
    // MotorSetRef(&(friction_motor.base), friction_motor_setref);
    // MotorSend(&(friction_motor.base));
}
