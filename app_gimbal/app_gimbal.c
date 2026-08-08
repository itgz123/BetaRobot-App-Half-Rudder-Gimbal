#include "app_gimbal.h"
#include "app_cfg.h"
//
#include "drv_motor_base.h"
#include "drv_dmmotor.h"
#include "drv_vofa.h"
#include "drv_axis_mit_lite.h"
#include "robot_def.h"

// yaw_motor; // yaw电机
DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机
static AxisMitLiteInstance pitchup_axis;

/* 私有函数 */
// pitchup方法
static void setpitchup(float ref)
{

    MotorSetRef(&(pitchup_motor.base), ref);
}
static MotorData_s getpitchup(void)
{
    MotorData_s data = MotorGetData(&(pitchup_motor.base));
    data.position = (pitchup_motor.base.data_all.data.position - pitchup_position_0) - (pitchdown_motor.base.data_all.data.position - pitchdown_position_min);
    return data;
}

/* 外部函数 */
void AppGimbalInit(void)
{
    // 注册 CAN 实例（下pitch、上pitch 共用 CAN_2）
    DMMotorRegister(&pitchdown_motor);
    DMMotorRegister(&pitchup_motor);

    // 配置下pitch
    DMMotor_Config_s pitchdown_cfg = {
        .can_e = CAN_2,
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
        .pos_max = M_PI,
        .t_range = 10,
        .vel_range = 30,
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
    };
    DMMotorConfig(&pitchdown_motor, &pitchdown_cfg);

    // 配置上pitch
    DMMotor_Config_s pitchup_cfg = {
        .can_e = CAN_2,
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
        .pos_max = M_PI,
        .t_range = 10,
        .vel_range = 30,
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
    };
    DMMotorConfig(&pitchup_motor, &pitchup_cfg);

    MotorEnable(&(pitchdown_motor.base));
    MotorEnable(&(pitchup_motor.base));

    AxisMitLite_Init_Config_s pitchup_axis_cfg = {
        .motor = {
            .get_data = getpitchup,
            .set_ref = setpitchup,
        },                                     // 电机接口 (set_ref/get_data)
        .stage = AXIS_LITE_STAGE_FIXED_TORQUE, // 控制阶段
        .delay_ms = 5000,                      // 延时时间 (ms)
        .params = {
            .gravity = 0.28,
            .gear_ratio = 1,
        },                        // 轴参数
        .sine_params = {0},       // 正弦参数
        .chirp_params = {0},      // 扫频参数
        .multi_sine_params = {0}, // 多正弦叠加参数
        .kp = 0,                  // 位置增益 (Nm/rad)
        .kd = 0,                  // 速度增益
    };
    AxisMitLiteInit(&pitchup_axis, &pitchup_axis_cfg);
}

ITCM_RAM void AppGimbalRun(void)
{
    // getdata,setref
    MotorGetData(&(pitchdown_motor.base));
    // MotorGetData(&(pitchup_motor.base));
    AxisMitLiteCalculate(&pitchup_axis);
    MotorSetRef(&(pitchdown_motor.base), 0);

    // send
    MotorSend(&(pitchdown_motor.base));
    MotorSend(&(pitchup_motor.base));

    VofaSend();
}
