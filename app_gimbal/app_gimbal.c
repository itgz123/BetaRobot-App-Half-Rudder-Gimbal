#include "app_gimbal.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h"
//
#include "drv_motor_base.h"
#include "drv_dmmotor.h"
#include "drv_rsmotor.h"
#include "drv_vofa.h"
#include "drv_axis_mit_lite.h"
//
#include "bsp_assert.h"

// 实例
DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机
RSMOTOR_INSTANCE_DEF(yaw_motor);       // yaw电机（RS05）
static AxisMitLiteInstance pitchup_axis;
static cmd2gimbal_data_t gimbal_cmd2gimbal_data; // cmd2gimbal

// 数据
static float pitchdown_motor_setref = 0;
static float pitchup_motor_setref = 0;
static float yaw_motor_setref = 0;

/* 外部函数 */
void AppGimbalInit(void)
{
    // 注册 CAN 实例（下pitch、上pitch 共用 CAN_2）
    BSP_ASSERT_APP_CALL(DMMotorRegister(&pitchdown_motor));
    BSP_ASSERT_APP_CALL(DMMotorRegister(&pitchup_motor));
    BSP_ASSERT_APP_CALL(RSMotorRegister(&yaw_motor));

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
        .speed_lpf_rc = 0.1,
        .pos_max = M_PI,
        .t_range = 10,
        .vel_range = 30,
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
    };
    BSP_ASSERT_APP_CALL(DMMotorConfig(&pitchdown_motor, &pitchdown_cfg));

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
        .speed_lpf_rc = 0.1,
        .pos_max = M_PI,
        .t_range = 10,
        .vel_range = 30,
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
    };
    BSP_ASSERT_APP_CALL(DMMotorConfig(&pitchup_motor, &pitchup_cfg));

    // 配置yaw（RS05，量程需与灵足上位机一致：位置±12.57rad/速度±50rad/s/力矩±5.5Nm）
    RSMotor_Config_s yaw_cfg = {
        .can_e = CAN_1,
        .controller_setting = {
            .loop_type = MOTOR_LOOP_OPEN,                          // 控制模式
            .feedback_direction = MOTOR_DIRECTION_NORMAL,          // 电机方向
            .motor_direction = MOTOR_DIRECTION_NORMAL,             // 反馈方向
            .position_mode = MOTOR_POSITION_WRAP,                  // 位置模式（yaw无限旋转用环绕）
            .angle_limit_max = M_PI,                               // WRAP: 归一化上限
            .angle_limit_min = -M_PI,                              // WRAP: 归一化下限
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针
        },
        .model = RS_MODEL_RS05,
        .can_id = 0x01,
        .master_id = 0xfd,
        .pid_angle_setting = {},
        .pid_speed_setting = {},
        .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
        .speed_lpf_rc = 0,
        .pos_max = 12.57f,  // RS05 默认量程
        .t_range = 5.5f,    // RS05 默认量程
        .vel_range = 50.0f, // RS05 默认量程
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
    };
    BSP_ASSERT_APP_CALL(RSMotorConfig(&yaw_motor, &yaw_cfg));

    MotorEnable(&(pitchdown_motor.base));
    MotorEnable(&(pitchup_motor.base));
    MotorEnable(&(yaw_motor.base));

    AxisMitLite_Init_Config_s pitchup_axis_cfg = {
        .stage = AXIS_LITE_STAGE_TUNE, // 控制阶段
        .delay_ms = 5000,              // 延时时间 (ms)
        .params = {
            .gravity = 0.28,
            .gear_ratio = 1,
            .inertia = 0.012f, // kg·m²
            .friction_coulomb_pos = 0.0f,
            .friction_coulomb_neg = 0.0f,
            .friction_viscous_pos = 0.0f, // Nm·s/rad
            .friction_viscous_neg = 0.0f,
        }, // 轴参数
        .sine_params = {
            .amplitude = 0.3,
            .freq = 2,
        }, // 正弦参数
        .chirp_params = {
            .amplitude_start = 0.1,
            .amplitude_end = 3,
            .duration = 15,
            .start_freq = 1,
            .end_freq = 8,
        }, // 扫频参数
        .multi_sine_params = {
            .amplitude = 0.1,
            .duration = 1,
            .num_freqs = 10,
        },       // 多正弦叠加参数
        .kp = 0, // 位置增益 (Nm/rad)
        .kd = 0, // 速度增益
    };
    BSP_ASSERT_APP_CALL(AxisMitLiteInit(&pitchup_axis, &pitchup_axis_cfg));
}

ITCM_RAM void AppGimbalRun(void)
{
    // 接收消息
    xQueueReceive(cmd2gimbal_queue_handle, &gimbal_cmd2gimbal_data, 0);

    // 计算当前状态
    MotorData_s pitchdown_mdata = MotorGetData(&(pitchdown_motor.base));
    MotorData_s pitchup_mdata = MotorGetData(&(pitchup_motor.base));
    pitchup_mdata.position = (pitchup_motor.base.data_all.data.position - pitchup_position_0) - (pitchdown_motor.base.data_all.data.position - pitchdown_position_min);
    MotorData_s yaw_mdata = MotorGetData(&(yaw_motor.base));

    // setref
    // 清零
    pitchup_motor_setref = 0;
    pitchdown_motor_setref = 0;
    yaw_motor_setref = 0;
    // setref-pitchup
    if (enable == gimbal_cmd2gimbal_data.state)
    {
        pitchup_motor_setref = AxisMitLiteCalculate(&pitchup_axis, &pitchup_mdata);
    }
    // setref-pitchdown
    if (enable == gimbal_cmd2gimbal_data.state)
    {
        // 固定值+重力前馈+速度误差项+pitchup力矩单向叠加
        float diejia_pitchup_motor_setref = 0; // 要叠加在pitchdown的力矩
        float pitchdown_kd = 0;
        if (pitchup_motor_setref < 0) // pitchup要单向的
        {
            diejia_pitchup_motor_setref = -pitchup_motor_setref;
        }
        if (pitchdown_mdata.position < pitchdown_position_max - 0.2) // 立起来就不要速度项了，但是在这个临界角度会问题
        {
            pitchdown_kd = 2;
        }
        pitchdown_motor_setref = 2.2 +                                                       // 固定值
                                 1.0 * (pitchdown_position_max - pitchdown_mdata.position) + // 约等于重力前馈
                                 pitchdown_kd * (2 - pitchdown_mdata.speed) +                // 速度误差项
                                 diejia_pitchup_motor_setref;                                // pitchup单向
    }
    // setref-yaw
    if (enable == gimbal_cmd2gimbal_data.state)
    {
        yaw_mdata = yaw_mdata; // 避免警告
        yaw_motor_setref = 0;
    }

    // send
    MotorSetRef(&(pitchup_motor.base), pitchup_motor_setref);
    MotorSetRef(&(pitchdown_motor.base), pitchdown_motor_setref);
    MotorSetRef(&(yaw_motor.base), yaw_motor_setref);
    MotorSend(&(pitchdown_motor.base));
    MotorSend(&(pitchup_motor.base));
    MotorSend(&(yaw_motor.base));

    // 其他
    VofaSetChannel(13, pitchdown_mdata.speed);
    VofaSetChannel(14, pitchdown_mdata.position);
    VofaSetChannel(15, pitchup_mdata.speed);
    VofaSend();
}
