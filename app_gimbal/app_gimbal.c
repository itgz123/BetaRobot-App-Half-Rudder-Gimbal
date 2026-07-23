#include "app_gimbal.h"
#include "app_cfg.h"
//
#include "drv_motor_base.h"
#include "drv_dmmotor.h"

// yaw_motor; // yaw电机
DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机
uint8_t temp = 0;
void AppGimbalInit(void)
{
    // 下pitch
    DMMotor_Register_Config_s pitchdown_config = {
        .can_e = CAN_2,
        .fault_action = DAEMON_FAULT_NONE,
        .reload_count = 5,
        .motor_config = {
            .controller_setting = {
                .loop_type = MOTOR_LOOP_OPEN,                          // 控制模式
                .feedback_direction = MOTOR_DIRECTION_NORMAL,          // 电机方向
                .motor_direction = MOTOR_DIRECTION_NORMAL,             // 反馈方向
                .position_mode = MOTOR_POSITION_LIMITED,               // 位置模式
                .angle_limit_max = 0,                                  // LIMITED: 限幅上限, WRAP: 归一化上限
                .angle_limit_min = 0,                                  // LIMITED: 限幅下限, WRAP: 归一化下限
                .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
                .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
                .speed_feedforward_ptr = NULL,                         // 速度前馈指针
                .position_feedforward_ptr = NULL,                      // 位置前馈指针
                .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
                .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
                .angle_external_ptr = NULL,                            // 外部角度反馈指针
                .speed_external_ptr = NULL,                            // 外部速度反馈指针
            },
            .model = DM_MODEL_DM4310,
            .can_id = 0x001,
            .master_id = 0x011,
            .pid_angle_setting = {},
            .pid_speed_setting = {},
            .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
            .speed_lpf_rc = 0,
            .pos_max = 12.5,
            .t_range = 3,
            .vel_range = 20,
        },
    };
    // 上pitch
    DMMotor_Register_Config_s pitchup_config = {
        .can_e = CAN_2,
        .fault_action = DAEMON_FAULT_NONE,
        .reload_count = 5,
        .motor_config = {
            .controller_setting = {
                .loop_type = MOTOR_LOOP_OPEN,                          // 控制模式
                .feedback_direction = MOTOR_DIRECTION_NORMAL,          // 电机方向
                .motor_direction = MOTOR_DIRECTION_NORMAL,             // 反馈方向
                .position_mode = MOTOR_POSITION_LIMITED,               // 位置模式
                .angle_limit_max = 0,                                  // LIMITED: 限幅上限, WRAP: 归一化上限
                .angle_limit_min = 0,                                  // LIMITED: 限幅下限, WRAP: 归一化下限
                .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
                .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
                .speed_feedforward_ptr = NULL,                         // 速度前馈指针
                .position_feedforward_ptr = NULL,                      // 位置前馈指针
                .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
                .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
                .angle_external_ptr = NULL,                            // 外部角度反馈指针
                .speed_external_ptr = NULL,                            // 外部速度反馈指针
            },
            .model = DM_MODEL_DM4310,
            .can_id = 0x002,
            .master_id = 0x012,
            .pid_angle_setting = {},
            .pid_speed_setting = {},
            .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
            .speed_lpf_rc = 0,
            .pos_max = 12.5,
            .t_range = 3,
            .vel_range = 20,
        },
    };
    if (0 == DMMotorRegister(&pitchdown_motor, &pitchdown_config))
    {
        temp += 1;
    }
    if (0 == DMMotorRegister(&pitchup_motor, &pitchup_config))
    {
        temp += 1;
    }
}

ITCM_RAM void AppGimbalRun(void)
{
    // MotorEnable((MotorBase_s *)(&pitchdown_motor));
    // MotorEnable((MotorBase_s *)(&pitchup_motor));
    // DMMotor_Enable((MotorBase_s *)(&pitchdown_motor));
    // DMMotor_Enable((MotorBase_s *)(&pitchup_motor));
}
