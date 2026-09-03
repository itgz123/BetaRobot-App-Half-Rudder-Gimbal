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
// #include "drv_bmi088.h"
// #include "lib_mahony.h"
//
#include "bsp_assert.h"

// 实例
DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机
RSMOTOR_INSTANCE_DEF(yaw_motor);       // yaw电机（RS05）
static AxisMitLiteInstance pitchup_axis;
static AxisMitLiteInstance yaw_axis;

// // 姿态传感器相关变量
// static BMI088_Data_t imu = {0};
// static euler_t euler = {0};
// static uint64_t last_imu_ts = 0; /* 上帧 IMU 时间戳 (us)，用于计算 dt */
// static float dt;
// static vector3_t gyro;
// static vector3_t acc;
// BMI088_INSTANCE_DEF(bmi088);
// MAHONY_INSTANCE_DEF(mahony);

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
        .timeout_ms = 1, // CAN 发送超时(ms)
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
        .speed_lpf_rc = 0.004, // 截止~40Hz, kp=40时ωn=11.3Hz总滞后140°裕度30° (电机8.6ms延迟限定了高kp必振荡)
        .pos_max = M_PI,
        .t_range = 10,
        .vel_range = 30,
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DMMotorConfig(&pitchup_motor, &pitchup_cfg));

    // 配置yaw（RS05，量程需与灵足上位机一致：位置±12.57rad/速度±50rad/s/力矩±5.5Nm）
    RSMotor_Config_s yaw_cfg = {
        .can_e = CAN_1,
        .controller_setting = {
            .loop_type = MOTOR_LOOP_OPEN,                          // 控制模式
            .feedback_direction = MOTOR_DIRECTION_REVERSE,         // 反馈方向：镜像后 逆时针→正角度（原编码器逆时针为负）
            .motor_direction = MOTOR_DIRECTION_REVERSE,            // 输出方向：镜像后 正力矩→逆时针（与反馈同步翻，闭环稳定）
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
        .speed_lpf_rc = 0.004f,
        .pos_max = 12.57f,  // RS05 默认量程
        .t_range = 5.5f,    // RS05 默认量程
        .vel_range = 50.0f, // RS05 默认量程
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(RSMotorConfig(&yaw_motor, &yaw_cfg));

    MotorEnable(&(pitchdown_motor.base));
    MotorEnable(&(pitchup_motor.base));
    MotorEnable(&(yaw_motor.base));

    AxisMitLite_Init_Config_s pitchup_axis_cfg = {
        .stage = AXIS_LITE_STAGE_NORMAL, // 控制阶段
        .delay_ms = 5000,                // 延时时间 (ms)
        .params = {
            .gravity = 0.30f, // 重力前馈系数（标定 0.28→0.30）
            .gear_ratio = 1,
            .inertia = 0.008f, // kg·m²（标定 0.012→0.008，前馈过大导致振幅放大）
            .friction_coulomb_pos = 0.0f,
            .friction_coulomb_neg = 0.0f,
            .friction_viscous_pos = 0.0f,
            .friction_viscous_neg = 0.0f,
        }, // 轴参数
        .sine_params = {
            .amplitude = 0.2,
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
        },         // 多正弦叠加参数
        .kp = 6,   // 位置增益 (Nm/rad)，电机延迟8.6ms限定kp上限, kp=80必振荡(16Hz位置环极限环), kp=40总滞后140°裕度30°
        .kd = 0.6, // 速度增益，配合RC=0.004(截止40Hz), kp=40时ζ≈0.88, 阻尼有效
    };
    BSP_ASSERT_APP_CALL(AxisMitLiteInit(&pitchup_axis, &pitchup_axis_cfg));

    AxisMitLite_Init_Config_s yaw_axis_cfg = {
        .stage = AXIS_LITE_STAGE_NORMAL, // 控制阶段
        .delay_ms = 5000,                // 延时时间 (ms)
        .vofa_enable = 1,                // 该轴写 VOFA 12 通道调试（多轴实例仅一个置 1）
        .params = {
            .gravity = 0.0f,
            .gear_ratio = 1,
            .inertia = 0.0095f,
            .friction_coulomb_pos = 0.0f,
            .friction_coulomb_neg = 0.0f,
            .friction_viscous_pos = 0.0f,
            .friction_viscous_neg = 0.0f,
        }, // 轴参数
        .sine_params = {
            .amplitude = 0.4,
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
        },         // 多正弦叠加参数
        .kp = 48,  // 位置增益
        .kd = 0.8, // 速度增益
        // yaw 是 WRAP 环绕轴（±π 归一化）：误差需取最短路径，否则边界处跳变
        .error_normalize_range = 2.0f * M_PI, // 误差 wrap 到 [-π, π)
        .error_normalize_enable = 1,          // 启用环绕误差归一化
        // TUNE 正弦参考默认以延时结束时的当前位置为中心（drv 层内置），避免起始误差过大
    };
    BSP_ASSERT_APP_CALL(AxisMitLiteInit(&yaw_axis, &yaw_axis_cfg));

    // // 注册 BMI088（只注册子模块，Config 时配置硬件）
    // BSP_ASSERT_APP_CALL(BMI088Register(&bmi088));

    // // 配置 BMI088（硬件枚举 + 传感器参数 + daemon）
    // BMI088_Config_s bmi088_cfg = {
    //     .spi_e = SPI_BMI088,
    //     .cs_acc_e = GPIO_BMI088_CS_ACCEL,
    //     .cs_gyro_e = GPIO_BMI088_CS_GYRO,
    //     .int_acc_e = GPIO_BMI088_INT_ACCEL,
    //     .int_gyro_e = GPIO_BMI088_INT_GYRO,
    //     .heater_e = TIM_HEATER,
    //     .daemon_reload = 20,
    //     .daemon_fault = DAEMON_FAULT_NONE,
    //     .acc_range = BMI088_ACC_RANGE_3G,
    //     .acc_bwp = BMI088_ACC_BWP_NORMAL,
    //     .acc_odr = BMI088_ACC_ODR_400,
    //     .gyro_range = BMI088_GYRO_RANGE_2000,
    //     .gyro_conf = BMI088_GYRO_CONF_2000_230,
    //     .work_mode = BMI088_MODE_INT,
    //     .spi_timeout_ms = 10, // SPI IT/DMA 传输超时(ms)
    // };
    // BSP_ASSERT_APP_CALL(BMI088Config(&bmi088, &bmi088_cfg));

    // // 初始化 Mahony 滤波器
    // Mahony_Init_Config_s mahony_cfg = {
    //     .kp = 0.5f,
    //     .ki = 0.0f,
    // };
    // MahonyInit(&mahony, &mahony_cfg);
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

    // // 读取 BMI088 原始数据
    // imu = BMI088ReadInt(&bmi088);

    // // 计算 dt：用 BMI088 插值时间戳之差 (us → s)
    // dt = 0.0f;
    // if (imu.time_stamp > 0 && last_imu_ts > 0)
    // {
    //     dt = (float)(imu.time_stamp - last_imu_ts) * 1e-6f;
    // }
    // last_imu_ts = imu.time_stamp;

    // // Mahony 姿态解算（dt 由 APP 层根据 BMI088 插值时间戳传入）
    // gyro.x = imu.gyro[0];
    // gyro.y = imu.gyro[1];
    // gyro.z = imu.gyro[2];
    // acc.x = imu.acc[0];
    // acc.y = imu.acc[1];
    // acc.z = imu.acc[2];
    // MahonyUpdate(&mahony, gyro, acc, dt);

    // // 从 Mahony 四元数解算 yaw 角
    // euler = Lib_Math_QuatToEuler(mahony.quat);

    // setref
    // 清零
    pitchup_motor_setref = 0;
    pitchdown_motor_setref = 0;
    yaw_motor_setref = 0;
    // setref-pitchup
    if (enable == gimbal_cmd2gimbal_data.state)
    {
        // 外部设定值来自 cmd（NORMAL 阶段使用；当前 TUNE 阶段内部正弦，此参数被忽略）
        AxisMitLiteRef_s pitchup_ref = {
            .position = gimbal_cmd2gimbal_data.pitch_x,
            .speed = gimbal_cmd2gimbal_data.pitch_v,
            .acceleration = gimbal_cmd2gimbal_data.pitch_a,
        };
        pitchup_motor_setref = AxisMitLiteCalculate(&pitchup_axis, &pitchup_mdata, &pitchup_ref);
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
        // 外部设定值来自 cmd（NORMAL 阶段使用；当前 TUNE 阶段内部正弦，此参数被忽略）
        AxisMitLiteRef_s yaw_ref = {
            .position = gimbal_cmd2gimbal_data.yaw_x,
            .speed = gimbal_cmd2gimbal_data.yaw_v,
            .acceleration = gimbal_cmd2gimbal_data.yaw_a,
        };
        yaw_motor_setref = AxisMitLiteCalculate(&yaw_axis, &yaw_mdata, &yaw_ref);
    }

    // send
    MotorSetRef(&(pitchup_motor.base), pitchup_motor_setref);
    MotorSetRef(&(pitchdown_motor.base), pitchdown_motor_setref);
    MotorSetRef(&(yaw_motor.base), yaw_motor_setref);
    MotorSend(&(pitchdown_motor.base));
    MotorSend(&(pitchup_motor.base));
    MotorSend(&(yaw_motor.base));

    // 其他
    // vofa发送
    VofaSend();

    // 回传云台反馈给 cmd（规划器需要当前位置/速度；pitch_down 供视觉回传下pitch位姿角）
    gimbal2cmd_data_t gimbal2cmd_data = {
        .pitch_position = pitchup_mdata.position,
        .pitch_vel = pitchup_mdata.speed,
        .yaw_position = yaw_mdata.position,
        .yaw_vel = yaw_mdata.speed,
        .pitch_down_position = pitchdown_mdata.position,
    };
    xQueueOverwrite(gimbal2cmd_queue_handle, &gimbal2cmd_data);
}
