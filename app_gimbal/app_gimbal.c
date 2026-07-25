#include "app_gimbal.h"
#include "app_cfg.h"
//
#include "drv_motor_base.h"
#include "drv_dmmotor.h"
#include "drv_vofa.h"

// yaw_motor; // yaw电机
DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机

void AppGimbalInit(void)
{
    // 注册 CAN 实例（下pitch、上pitch 共用 CAN_2）
    DMMotorRegister(&pitchdown_motor, CAN_2);
    DMMotorRegister(&pitchup_motor, CAN_2);

    // 配置下pitch
    DMMotor_Config_s pitchdown_cfg = {
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
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
    };
    DMMotorConfig(&pitchdown_motor, &pitchdown_cfg);

    // 配置上pitch
    DMMotor_Config_s pitchup_cfg = {
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
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
    };
    DMMotorConfig(&pitchup_motor, &pitchup_cfg);

    MotorEnable((MotorBase_s *)(&pitchdown_motor));
    MotorEnable((MotorBase_s *)(&pitchup_motor));
}

ITCM_RAM void AppGimbalRun(void)
{
    MotorGetData((MotorBase_s *)(&pitchdown_motor));
    MotorGetData((MotorBase_s *)(&pitchup_motor));
    VofaSetChannel(1, pitchup_motor.base.data.position);
    VofaSetChannel(2, pitchup_motor.base.data.position_single);
    VofaSetChannel(3, pitchup_motor.base.data.speed);
    VofaSetChannel(4, pitchup_motor.base.data.torque_current);

    VofaSetChannel(5, pitchdown_motor.base.data.position);
    VofaSetChannel(6, pitchdown_motor.base.data.position_single);
    VofaSetChannel(7, pitchdown_motor.base.data.speed);
    VofaSetChannel(8, pitchdown_motor.base.data.torque_current);

    VofaSend();
}
