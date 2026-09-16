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
#include "drv_terminal_lite.h"
#include "drv_bmi088.h"
#include "lib_mahony.h"
//
#include "bsp_assert.h"

// 实例
DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机
RSMOTOR_INSTANCE_DEF(yaw_motor);       // yaw电机（RS05）
static AxisMitLiteInstance pitchup_axis;
static AxisMitLiteInstance yaw_axis;

// 姿态传感器相关变量
/* 用多速率接口读原始数据：acc/gyro 各带自己的时间戳，不做插值对齐。
 * dt 由陀螺仪自身相邻两帧时间戳差分得到，与下面参与积分的 gyro 样本严格同源；
 * acc 只做低频姿态校正，最多滞后一个 acc 周期（400Hz → 2.5ms），对校正量无影响 */
static BMI088_MultiRateData_t imu = {0};
static euler_t euler = {0};
static uint64_t last_gyro_ts = 0; /* 上帧陀螺仪时间戳 (us)，用于计算 dt */
static float dt;
static vector3_t gyro;
static vector3_t acc;

/* dt 上限(s)：任务被拖延（app.c 会打 DELAY 日志）时限制单步积分步长，
 * 避免一次大 dt 让四元数一步跳变 */
#define IMU_DT_MAX_S (0.01f)

/* 陀螺仪零偏 (rad/s)，静止 55s 实测（本机 IMU 倾斜安装：roll -1.78° pitch +15.39°）：
 *   gx +0.00089  gy -0.00011  gz +0.00169   →  +0.051 / -0.006 / +0.097 °/s
 *
 * 为什么必须补偿：六轴 Mahony 的 yaw 完全来自陀螺积分，加速度计对它没有任何
 * 校正能力（交叉积误差里没有 yaw 分量），不补偿就是纯漂移。补偿前后实测：
 *   yaw 漂移  +0.64 °/min → +0.04 °/min；roll/pitch 静差 -0.20/-0.25° → <0.02°
 *   （静差来自 2*bias/kp，kp=1 时 bias 直接按 2 倍体现在倾角上）
 * 留出验证：用前半段（升温中）标定、后半段（工作点）评估，仍有 +0.05 °/min，
 * 说明这组值不是过拟合到某一段热状态。
 *
 * ⚠ 这组值是在加热器工作、温度稳定在 ~50°C 时测的，加热器目标温度也是 50°C。
 *   gz（决定 yaw 的那根轴，占 yaw 轴投影的 96%）实测温度系数 ≈ 0（r=0.000），
 *   所以定值补偿很稳；gx/gy 有约 -0.006 °/s/°C 的温度系数，但 gy 对 yaw 的
 *   耦合只有 sin(roll)≈3%，gx 只影响 roll 静差（±2°C 波动 → 0.02°），可忽略，
 *   故不做温度补偿。
 * 换 IMU / 拆装 / 改动加热策略后必须重新标定。 */
static const float s_gyro_offset[3] = {0.00089f, -0.00011f, 0.00169f};

/* VOFA 调参用派生量（只在 AppGimbalRun 里更新，仅供观察，不参与控制） */
static float yaw_unwrapped = 0.0f;  // yaw 解缠值 (rad)：转大角度时不在 ±π 处跳变
static float acc_norm = 0.0f;       // 合加速度模长 (m/s^2)：静止应 ≈ 9.80665
static float acc_roll_err = 0.0f;   // roll 校正残差 (rad)：euler.roll - 加速度反算 roll
static float acc_pitch_err = 0.0f;  // pitch 校正残差 (rad)
static float imu_temp = 0.0f;       // BMI088 温度 (℃)：陀螺仪零偏随温度漂移
static uint8_t attitude_seeded = 0; // 上电姿态是否已用加速度计播种
BMI088_INSTANCE_DEF(bmi088);
MAHONY_INSTANCE_DEF(mahony);

static cmd2gimbal_data_t gimbal_cmd2gimbal_data; // cmd2gimbal

// 数据
static float pitchdown_motor_setref = 0;
static float pitchup_motor_setref = 0;
static float yaw_motor_setref = 0;

/* 串口调参（terminal_lite）：type=本次操作 CMD_GET/CMD_SET（单一操作，非掩码 3）；
 * set 值已由模块按命令表 min/max 校验过，exec 只做读写 */
static int8_t GimbalParamExec(CmdType_e type, float value, float *pval, float *var)
{
    if (type == CMD_SET)
        *var = value;
    *pval = *var; // 读=当前值；写=写入后的值（供默认回显）
    return 0;
}

static int8_t ExecPupKp(CmdType_e type, float v, float *p) { return GimbalParamExec(type, v, p, &pitchup_axis.mit.kp); }
static int8_t ExecPupKd(CmdType_e type, float v, float *p) { return GimbalParamExec(type, v, p, &pitchup_axis.mit.kd); }
static int8_t ExecYawKp(CmdType_e type, float v, float *p) { return GimbalParamExec(type, v, p, &yaw_axis.mit.kp); }
static int8_t ExecGrav(CmdType_e type, float v, float *p) { return GimbalParamExec(type, v, p, &pitchup_axis.params.gravity); }
static int8_t ExecStage(CmdType_e type, float value, float *pval)
{
    (void)value;
    if (type == CMD_SET)
        return -1; // 只读
    *pval = (float)pitchup_axis.stage;
    return 0;
}

/* 命令表：结构 {token, type, scale, min, max, exec}。
 * scale 默认回显定点化倍率（无 %f，val*scale 转 int32 后 %d 回显，如千分位=1000 → 4.5 回 "4500"）；
 * min/max 为 tlset 写入上下限（含），越限回 err；无限制用 TERMINAL_LITE_NO_BOUND。 */
static TerminalLiteCmd_s s_gimbal_tl_cmds[] = {
    {"pup_kp", CMD_GET_SET, 1000, 0.0f, 100.0f, ExecPupKp}, // 例：读/写 pitchup 轴 kp
    {"pup_kd", CMD_GET_SET, 1000, 0.0f, 100.0f, ExecPupKd},
    {"yaw_kp", CMD_GET_SET, 1000, 0.0f, 100.0f, ExecYawKp},
    {"grav", CMD_GET_SET, 1000, -10.0f, 10.0f, ExecGrav},
    {"stage", CMD_GET, 1, TERMINAL_LITE_NO_BOUND, ExecStage}, // 只读示例（整数）
};
#define GIMBAL_TL_CMD_NUM (sizeof(s_gimbal_tl_cmds) / sizeof(s_gimbal_tl_cmds[0]))

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
        .kp = 4,   // 位置增益 (Nm/rad)，电机延迟8.6ms限定kp上限, kp=80必振荡(16Hz位置环极限环), kp=40总滞后140°裕度30°
        .kd = 0.4, // 速度增益，配合RC=0.004(截止40Hz), kp=40时ζ≈0.88, 阻尼有效
    };
    BSP_ASSERT_APP_CALL(AxisMitLiteInit(&pitchup_axis, &pitchup_axis_cfg));

    AxisMitLite_Init_Config_s yaw_axis_cfg = {
        .stage = AXIS_LITE_STAGE_NORMAL, // 控制阶段
        .delay_ms = 5000,                // 延时时间 (ms)
        // .vofa_enable = 1,                // 该轴写 VOFA 12 通道调试（多轴实例仅一个置 1）
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
        .kp = 1,   // 位置增益
        .kd = 0.1, // 速度增益
        // yaw 是 WRAP 环绕轴（±π 归一化）：误差需取最短路径，否则边界处跳变
        .error_normalize_range = 2.0f * M_PI, // 误差 wrap 到 [-π, π)
        .error_normalize_enable = 1,          // 启用环绕误差归一化
        // TUNE 正弦参考默认以延时结束时的当前位置为中心（drv 层内置），避免起始误差过大
    };
    BSP_ASSERT_APP_CALL(AxisMitLiteInit(&yaw_axis, &yaw_axis_cfg));

    // 注册 BMI088（只注册子模块，Config 时配置硬件）
    BSP_ASSERT_APP_CALL(BMI088Register(&bmi088));
    // 配置 BMI088（硬件枚举 + 传感器参数 + daemon）
    BMI088_Config_s bmi088_cfg = {
        .spi_e = SPI_BMI088,
        .cs_acc_e = GPIO_BMI088_CS_ACCEL,
        .cs_gyro_e = GPIO_BMI088_CS_GYRO,
        .int_acc_e = GPIO_BMI088_INT_ACCEL,
        .int_gyro_e = GPIO_BMI088_INT_GYRO,
        .heater_e = TIM_HEATER,
        .daemon_reload = 20,
        .daemon_fault = DAEMON_FAULT_NONE,
        .acc_range = BMI088_ACC_RANGE_3G,     // ±3g，云台不会有大加速度，取最小量程换分辨率
        .acc_bwp = BMI088_ACC_BWP_NORMAL,     // 正常带宽（ODR>400Hz 才必须切 OSR 模式）
        .acc_odr = BMI088_ACC_ODR_400,        // 400Hz：只喂 Mahony 的低频校正，够用
        .gyro_range = BMI088_GYRO_RANGE_2000, // ±2000dps，留余量避免大机动削顶
        /* ODR=1000Hz/BW=116Hz：任务周期 2ms(500Hz)，2000Hz 的采样用不上，
         * 反而把 250Hz 以上的噪声折返进来；1000Hz 配 116Hz 带宽噪声更低 */
        .gyro_conf = BMI088_GYRO_CONF_1000_116,
        .work_mode = BMI088_MODE_INT,
        .spi_timeout_ms = 10,         // SPI IT/DMA 传输超时(ms)
        .gyro_offset = s_gyro_offset, // 静止标定零偏，见 s_gyro_offset 定义
    };
    BSP_ASSERT_APP_CALL(BMI088Config(&bmi088, &bmi088_cfg));
    // 初始化 Mahony 滤波器
    /* kp：加速度计校正增益。实现里 gyro += kp * halfex，halfv 只有旋转矩阵的一半，
     *     故等效增益为 kp/2（kp=1.0 ≈ Madgwick 默认 Kp=0.5），误差时间常数约 2s。
     *     实测加速度计角度噪声仅 0.14°(rms)，kp 提到 2~5 噪声代价仍可忽略，
     *     但大机动时加速度计不可信，宁可信陀螺仪，故维持 1.0。
     * ki：陀螺仪零偏估计。置 0 —— 六轴 yaw 不可观测，ki 的 z 分量没有可信的
     *     校正源，反而会把加速度计噪声积成假零偏。改用实测零偏硬补偿，
     *     确定性更好（见 s_gyro_offset）。 */
    Mahony_Init_Config_s mahony_cfg = {
        .kp = 1.0f,
        .ki = 0.0f,
    };
    MahonyInit(&mahony, &mahony_cfg);

    // 串口调参终端（轴实例就绪后注册命令表）
    TerminalLiteInit(s_gimbal_tl_cmds, (uint8_t)GIMBAL_TL_CMD_NUM);
}

/**
 * @brief 把 wrap 到 (-π, π] 的 yaw 展开成连续值
 * @param yaw 当前 yaw (rad)
 * @return 解缠后的 yaw (rad)，从首次调用的位置起累计，转多圈也连续
 * @note 相邻两帧取最短路径增量再累加，跨 ±π 时不跳变。
 *       调参采集 yaw 旋转数据时用它，否则曲线会在 ±π 处被折断
 */
static float UnwrapYaw(float yaw)
{
    static float yaw_prev = 0.0f;
    static float yaw_acc = 0.0f;
    static uint8_t init = 0;

    if (!init)
    {
        yaw_prev = yaw;
        yaw_acc = yaw;
        init = 1;
    }
    else
    {
        yaw_acc += Lib_Math_WrapAngleNegPIToPI(yaw - yaw_prev);
        yaw_prev = yaw;
    }

    return yaw_acc;
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

    // 换算为 axis lite 的反馈输入（lite 层不依赖 motor，故在 app 侧剥离 MotorData_s）
    AxisLiteState_s pitchup_state = {
        .position = (float)pitchup_mdata.position,
        .speed = pitchup_mdata.speed,
        .torque = pitchup_mdata.torque,
    };

    // 读取 BMI088 原始数据（陀螺仪/加速度计各带独立时间戳，无插值）
    imu = BMI088ReadLatest(&bmi088);

    // 计算 dt：陀螺仪相邻两帧时间戳之差 (us → s)
    dt = 0.0f;
    if (imu.time_stamp_g > 0 && last_gyro_ts > 0)
    {
        dt = (float)(imu.time_stamp_g - last_gyro_ts) * 1e-6f;
        if (dt > IMU_DT_MAX_S)
            dt = IMU_DT_MAX_S; // 任务被拖长时的保护
    }
    if (imu.time_stamp_g > 0)
        last_gyro_ts = imu.time_stamp_g; // 数据未就绪(0)时不更新，恢复后本帧 dt=0 自然跳过

    // Mahony 姿态解算（dt 由 APP 层根据陀螺仪时间戳传入）
    gyro.x = imu.gyro[0];
    gyro.y = imu.gyro[1];
    gyro.z = imu.gyro[2];
    acc.x = imu.acc[0];
    acc.y = imu.acc[1];
    acc.z = imu.acc[2];

    /* 上电姿态播种：用第一帧可信的加速度计反算 roll/pitch 直接写进四元数。
     * 不播种的话滤波器从单位四元数慢慢收敛（kp=1 → τ≈2s，实测要约 6~10s），
     * 这段过程里 ① 姿态全是错的 ② 加速度计校正的 z 分量会把 yaw 顺带推偏约 3°。
     * yaw 无法从加速度计观测，取 0 作为航向基准。 */
    acc_norm = Lib_Math_Vec3Length(acc);
    if (!attitude_seeded && imu.time_stamp_g > 0 && Lib_Math_Fabs(acc_norm - 9.80665f) < 2.0f)
    {
        attitude_seeded = 1;
        euler_t init_e = {
            .roll = Lib_Math_Atan2(acc.y, acc.z),
            .pitch = Lib_Math_Atan2(-acc.x, Lib_Math_Sqrt(acc.y * acc.y + acc.z * acc.z)),
            .yaw = 0.0f,
        };
        mahony.quat = Lib_Math_EulerToQuat(init_e);
    }

    MahonyUpdate(&mahony, gyro, acc, dt);

    // 从 Mahony 四元数解算欧拉角 (rad)
    euler = Lib_Math_QuatToEuler(mahony.quat);

    /* ---- yaw 轴反馈：取自 IMU（世界系），不用电机编码器 ----
     * IMU 刚性固定在云台底座（yaw 轴输出、两个 pitch 关节的上游）：
     *   ① 两个 pitch 轴怎么转都带不动 IMU，只有 yaw 轴转动会改变它的姿态；
     *   ② 因此它报告的 roll/pitch 就是恒定的安装倾角（-1.78° / +15.39°），
     *      euler.yaw 就是 yaw 轴的世界系航向，与视觉下发的世界系 yaw 同一坐标系。
     * 位置直接用 euler.yaw。
     * 速度把机体系陀螺仪投影到世界 Z 轴：世界 Z 轴在 IMU 机体系下的分量是 R 的第三行
     *   [-sinθ, cosθ·sinφ, cosθ·cosφ]   (θ=pitch, φ=roll)，该向量为单位向量，
     * 故 ψ̇ = 上式 · gyro 精确成立。不能直接取 gyro.z：安装倾角 15.39° 使
     * cosθ·cosφ≈0.963，gyro.z 只有真实航向角速度的 96%，另外还有 -sinθ·ψ̇ 落在 gyro.x 上。
     * 为什么不用编码器：编码器给的是"相对底盘"的关节角，底盘自转/被推动/回差时
     * 都不等于云台的真实指向；要控世界系航向就得拿世界系的量来控。 */
    float yaw_rate_world = -Lib_Math_Sin(euler.pitch) * gyro.x +
                           Lib_Math_Cos(euler.pitch) * Lib_Math_Sin(euler.roll) * gyro.y +
                           Lib_Math_Cos(euler.pitch) * Lib_Math_Cos(euler.roll) * gyro.z;
    AxisLiteState_s yaw_state = {
        .position = euler.yaw,
        .speed = yaw_rate_world,
        .torque = yaw_mdata.torque,
    };

    /* ---- VOFA 调参派生量 ---- */
    yaw_unwrapped = UnwrapYaw(euler.yaw);
    /* 由加速度计直接反算的 roll/pitch，与 lib_mahony 的 halfv 取同一约定
     * （静止时 accel 指向 +Z、模长 1g）。静止且 acc 可信时，euler 与它之差 ≈ 0；
     * 若残差 ≈ 2×euler，说明 acc 的符号约定与库不一致（校正会往反方向推）。
     * 残差也约等于 kp*error/2 的稳态余量，是调 kp 最直接的观测量 */
    {
        float acc_roll = Lib_Math_Atan2(acc.y, acc.z);
        float acc_pitch = Lib_Math_Atan2(-acc.x, Lib_Math_Sqrt(acc.y * acc.y + acc.z * acc.z));
        acc_roll_err = Lib_Math_WrapAngleNegPIToPI(euler.roll - acc_roll);
        acc_pitch_err = euler.pitch - acc_pitch;
    }
    imu_temp = bmi088.temperature;

    // setref
    /* 状态机：stop = 失能（setref 保持 0）；normal/gyro 两档云台行为一致，给定都由 cmd
     * 算好，差异只体现在底盘 w 上、且已由 cmd 侧按模式算完，故这里统一判 != stop。
     * hole（过洞）：pitch 上下两个电机输出恒为 0（电机保持使能、仍发零力矩控制帧，
     * 云台靠机构自重/收拢下垂），只有 yaw 继续控世界系航向。 */
    robot_mode gimbal_mode = gimbal_cmd2gimbal_data.mode;
    uint8_t pitch_off = (robot_mode_hole == gimbal_mode); // 过洞模式关闭 pitch
    // 清零
    pitchup_motor_setref = 0;
    pitchdown_motor_setref = 0;
    yaw_motor_setref = 0;
    // setref-pitchup
    if (robot_mode_stop != gimbal_mode && !pitch_off)
    {
        // 外部设定值来自 cmd（NORMAL 阶段使用；当前 TUNE 阶段内部正弦，此参数被忽略）
        AxisMitLiteRef_s pitchup_ref = {
            .position = gimbal_cmd2gimbal_data.pitch_x,
            .speed = gimbal_cmd2gimbal_data.pitch_v,
            .acceleration = gimbal_cmd2gimbal_data.pitch_a,
        };
        pitchup_motor_setref = AxisMitLiteCalculate(&pitchup_axis, &pitchup_state, &pitchup_ref);
    }
    // setref-pitchdown
    if (robot_mode_stop != gimbal_mode && !pitch_off)
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
    if (robot_mode_stop != gimbal_mode)
    {
        // 外部设定值来自 cmd（NORMAL 阶段使用；当前 TUNE 阶段内部正弦，此参数被忽略）
        AxisMitLiteRef_s yaw_ref = {
            .position = gimbal_cmd2gimbal_data.yaw_x,
            .speed = gimbal_cmd2gimbal_data.yaw_v,
            .acceleration = gimbal_cmd2gimbal_data.yaw_a,
        };
        yaw_motor_setref = AxisMitLiteCalculate(&yaw_axis, &yaw_state, &yaw_ref);
    }

    // send
    MotorSetRef(&(pitchup_motor.base), pitchup_motor_setref);
    MotorSetRef(&(pitchdown_motor.base), pitchdown_motor_setref);
    MotorSetRef(&(yaw_motor.base), yaw_motor_setref);
    MotorSend(&(pitchdown_motor.base));
    MotorSend(&(pitchup_motor.base));
    MotorSend(&(yaw_motor.base));

    // 串口调参：处理收到的命令（每帧最多 8 条，空队列时几乎零开销）
    TerminalLiteExecute(1);

    // 其他
    // vofa发送
    /* VOFA 调试通道（CH0=时间戳由驱动自动填充，数据在 VofaSend 时发出）：
     *   CH1-3 : euler roll/pitch/yaw (rad)，yaw 为 wrap 到 (-π,π] 的原始值
     *   CH4-6 : 陀螺仪 gyro x/y/z (rad/s)
     *   CH7-9 : 加速度计 acc x/y/z (m/s^2)
     *   CH10  : dt (ms)，可与 CH0 时间戳对照检查采样是否连续
     *   CH11  : 合加速度 |acc| (m/s^2)，静止应 ≈ 9.80665；
     *           偏离 9.80665±0.3 时 lib_mahony 会整帧跳过加速度计校正
     *   CH12  : yaw 解缠值 (rad)，转多圈连续，采集 yaw 旋转数据时用它而不是 CH3
     *   CH13  : BMI088 温度 (℃)，陀螺仪零偏随温度漂移，可用来判断静态零偏是否有效
     *   CH14  : roll 校正残差 (rad) = euler.roll - 加速度反算 roll，
     *           静止收敛后应 ≈ 0（只剩 2*bias/kp 的稳态余量，零偏补偿后应 < 0.01°）；
     *           若残差 ≈ 2×euler.roll 说明 halfv 或 acc 符号约定错了
     *   CH15  : pitch 校正残差 (rad)，同上
     *
     * 调参用法：
     *   静态采集 —— 用 CH4-6 求陀螺仪零偏(均值)与噪声(标准差)，CH11 确认 acc 可信，
     *               CH1-2/CH14-15 看 roll/pitch 是否收敛且无静差，CH3/CH12 看 yaw 漂移速率
     *   转动 yaw —— 底盘固定只转 yaw，用 CH12 与 cumsum(CH6*CH10/1000) 对比。
     *               注意：IMU 倾斜安装时两者本来就不相等，比值 = 1/cos(pitch)
     *               （本机 pitch≈16° → 1.04），这是欧拉角运动学耦合、不是标度误差；
     *               要核标度就把 CH12 与"陀螺在转动轴上的投影积分"比。
     *               同时 CH1-2/CH14-15 应基本不动
     *
     * 注意：drv_axis_mit_lite 的 vofa_enable 同样占用 CH1~CH12，
     *       若后续要开某个轴的调试输出，需先让出/错开这些通道 */
    // VofaSetChannel(1, euler.roll);
    // VofaSetChannel(2, euler.pitch);
    // VofaSetChannel(3, euler.yaw);
    // VofaSetChannel(4, gyro.x);
    // VofaSetChannel(5, gyro.y);
    // VofaSetChannel(6, gyro.z);
    // VofaSetChannel(7, acc.x);
    // VofaSetChannel(8, acc.y);
    // VofaSetChannel(9, acc.z);
    // VofaSetChannel(10, dt * 1000.0f);
    // VofaSetChannel(11, acc_norm);
    // VofaSetChannel(12, yaw_unwrapped);
    // VofaSetChannel(13, imu_temp);
    // VofaSetChannel(14, acc_roll_err);
    // VofaSetChannel(15, acc_pitch_err);
    // VofaSend();

    /* 回传云台反馈给 cmd（规划器需要当前位置/速度；pitch_down 供视觉回传下pitch位姿角）。
     * yaw 与 yaw_vel 回传的是 IMU 世界系量：必须与上面 yaw 轴控制用的反馈同源，
     * 否则 cmd 的规划器会拿"相对底盘"的锚点去规划"世界系"的目标，每帧都差一个底盘航向。
     * 同时视觉下发的 yaw 本就是世界系（见 app_cmd 的 gimbal_from_vision），回传也对齐。
     * yaw_motor_* 另外回传编码器关节角（世界系量丢掉了"云台相对底盘"这一信息）：
     *   ⚠ cmd 侧 chassis_w_from_mode 的底盘跟随 w = kp*wrap(-yaw_position) 要的是相对角，
     *     现在拿的是世界系航向，normal/gyro 下会持续自转。这两个字段就是给它预留的，
     *     cmd 侧切过来后，底盘跟随改用 yaw_motor_position/yaw_motor_vel 即可。 */
    gimbal2cmd_data_t gimbal2cmd_data = {
        .pitch_position = pitchup_mdata.position,
        .pitch_vel = pitchup_mdata.speed,
        .yaw_position = euler.yaw,
        .yaw_vel = yaw_rate_world,
        .pitch_down_position = pitchdown_mdata.position,
        .yaw_motor_position = (float)yaw_mdata.position,
        .yaw_motor_vel = yaw_mdata.speed,
    };
    xQueueOverwrite(gimbal2cmd_queue_handle, &gimbal2cmd_data);
}
