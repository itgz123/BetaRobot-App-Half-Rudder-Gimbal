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
#include "drvlib_bmi088_kalman.h"
//
#include "bsp_sys_status.h"
//
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "drv_ist8310.h"

/* ═══════════════ IST8310 磁力计 bring-up 测试（临时，验完整块删掉） ═══════════════
 *
 * 目的：只做一件事 —— 确认 bsp_i2c + drv_ist8310 的读取链路能拿到合理地磁。
 * 当前配置是 IST8310_MODE_POLLING + I2C_DMA_MODE：采样仍由任务轮询驱动（不碰 DRDY 中断），
 * 但每笔 I2C 走 DMA，传输期间 CPU 空闲。**调用方看到的接口完全没变** ——
 * IST8310ReadBlocking 依旧是"调完就拿到新数据"，异步等待被封在驱动内部。
 *
 * 两个必须知道的点：
 *
 * 1) **`IST8310Config` 不能放在 AppGimbalInit 里。**
 *    `function_in_main_c()` 用 `__disable_irq()` 把整个初始化段包住了，而 HAL 的
 *    阻塞式 I2C 用 `HAL_GetTick()` 计时。以 F4 的 `I2C_WaitOnFlagUntilTimeout` 为例：
 *        while (flag == Status) { if ((HAL_GetTick() - Tickstart) > Timeout) {...} }
 *    中断关着时 tick 不前进，那个 if 永远不成立 —— 只要标志一直不来就是**死循环**。
 *    而 `HAL_I2C_Mem_Read` 一进来就在等 BUSY 清零：SDA/SCL 接反、器件没供电、
 *    上一笔传输被中途打断，都会让总线停在 BUSY，启动阶段直接卡死、连日志都出不来。
 *    所以这里把 Config 推迟到第一次任务调用时做（那时中断已开、超时正常生效，
 *    出错会规规矩矩返回 -1 并走恢复链）。`IST8310Register` 只做实例登记、不碰硬件，
 *    放 Init 里是安全的。
 *
 * 2) **本测试会拖慢云台任务，所以做了分频。**
 *    一次阻塞读 ≈ 测量延时 6ms + 一次 I2C 往返，而云台任务周期只有 2ms，
 *    每个周期都读必然触发 APP_TASK_DEF 的 DELAY 上报。按 IST8310_TEST_DIV 分频到
 *    100ms 一帧；验证时电机是停的，这点抖动无所谓。
 *
 * 看结果：本板 app_cfg.h 里 `LOG_UART` 是注释掉的，BSPLOG 被编成空宏 ——
 * 所以现在**只能靠调试器看下面的全局变量**（launch.json 的 live watch 里加
 * g_ist8310_test_*）。把 LOG_UART 打开后日志也会一起出来。
 */
#ifdef DRV_IST8310_USED

/* 本模块日志实例（app.c 的 g_app_log 是 static，这里另开一个）。
 * 放在守卫里：眼下只有本测试块用日志，开关关掉后留着它会变成未使用变量 */
LOG_INSTANCE_DEF(g_gimbal_log, "gimbal", 20);

IST8310_INSTANCE_DEF(ist8310);

#define IST8310_TEST_DIV 50        // 每 50 个任务周期读一帧 = 50 × 2ms = 100ms
#define IST8310_TEST_TIMEOUT_MS 10 // 单笔 I2C 超时(ms)

/* 非 static：供调试器直接观察。本板日志被编掉时**这里是唯一的诊断口**，
 * 所以状态特意分成了三态 —— 光看"读数为 0"分不清是没初始化还是初始化失败 */
IST8310_Data_t g_ist8310_test_data; // 最近一次成功的读数 (µT)
uint32_t g_ist8310_test_ok = 0;     // 累计成功帧数
uint32_t g_ist8310_test_fail = 0;   // 累计失败次数
uint32_t g_ist8310_test_us = 0;     // 最近一次阻塞读耗时 (µs)
/* Config 状态：0 = 还没做；1 = 成功（已开始采样）；2 = 失败（不再重试，查接线/供电/上拉） */
volatile uint8_t g_ist8310_test_state = 0;

static const IST8310_Config_s s_ist8310_test_cfg = {
    .i2c_e = I2C_IST8310, // DJI_C: I2C3 (PA8=SCL / PC9=SDA)
    /* 轮询模式用不到 DRDY。这里填哨兵值表示"未接"：驱动会跳过该 GPIO 的配置，
     * 不会白占一个 EXTI 槽位（将来切实测中断模式时再填 GPIO_IST8310_DRDY） */
    .drdy_e = GPIO_NUM_MAX,
    .rstn_e = GPIO_IST8310_RSTN, // PG6，接了就填；恢复流程会顺带脉冲它
    .daemon_reload = 200,        // 100ms 一帧，留 2 倍余量（daemon 任务 1ms 减一次）
    .daemon_fault = DAEMON_FAULT_NONE,
    .work_mode = IST8310_MODE_POLLING,
    /* 传输走 DMA（CubeMX 侧 I2C3 的 TX/RX DMA 已配好，DMA1_Stream2/Stream4 中断已使能）。
     * 本板 app_cfg.h 里 LOG_UART 是注释掉的，DMA 链路的问题不会出现在日志里 ——
     * 只能靠 g_ist8310_test_* 那几个全局变量看：读数为真但 g_ist8310_test_us 明显变大，
     * 或 g_ist8310_test_fail 开始涨，都是 DMA 没跑起来的征兆（超时会走到驱动的
     * "Async xfer timeout" 分支，计时用的是 i2c_timeout_ms） */
    .i2c_mode = I2C_DMA_MODE,
    .avg = IST8310_AVG_16,               // 手册推荐的低噪声设置（两次测量最小间隔 6ms）
    .pd_pulse = IST8310_PD_PULSE_NORMAL, // 手册 §3.1.1：初始配置必须写 0xC0
    /* 轮询模式没有 DRDY 中断，极性只决定写进 CNTL2 的 DRP 位。
     * ⚠ PG3 的 EXTI 配的是上升沿触发，将来真的切中断模式时这里必须是 ACTIVE_HIGH */
    .drdy_polarity = IST8310_DRDY_ACTIVE_HIGH,
    .i2c_timeout_ms = IST8310_TEST_TIMEOUT_MS,
};

/**
 * @brief IST8310 阻塞读测试（任务上下文，每 IST8310_TEST_DIV 个周期跑一次）
 */
static void IST8310_TestPolling(void)
{
    /* 首次调用：做 Config（含 POR 等待与阻塞 I2C，约 100ms+，会触发一次 DELAY 上报） */
    if (g_ist8310_test_state == 0)
    {
        if (IST8310Config(&ist8310, &s_ist8310_test_cfg) != 0)
        {
            g_ist8310_test_state = 2; // 失败就不反复重试了，避免每 2ms 刷一次日志
            BSPLOG(&g_gimbal_log, LOG_LEVEL_ERROR, "IST8310 config failed, check wiring/power");
            return;
        }
        g_ist8310_test_state = 1;
        BSPLOG(&g_gimbal_log, LOG_LEVEL_INFO, "IST8310 config ok (polling + dma, avg=16)");
        return;
    }
    if (g_ist8310_test_state == 2)
    {
        return; // 已放弃
    }

    static uint32_t div = 0;
    if (++div < IST8310_TEST_DIV)
    {
        return;
    }
    div = 0;

    uint64_t t0 = DWT_GetTimeUs();
    IST8310_Data_t data = IST8310ReadBlocking(&ist8310);
    g_ist8310_test_us = (uint32_t)(DWT_GetTimeUs() - t0);

    /* 约定的失败表示：time_stamp == 0（见 drv_ist8310.h 的返回值说明） */
    if (data.time_stamp == 0)
    {
        g_ist8310_test_fail++;
        BSPLOG(&g_gimbal_log, LOG_LEVEL_WARNING, "IST8310 read fail (online=%d, fail=%d)",
               (int)IST8310IsOnline(&ist8310), (int)g_ist8310_test_fail);
        return;
    }

    g_ist8310_test_data = data;
    g_ist8310_test_ok++;

    /* 判断依据：地磁总场约 25~65 µT（水平分量 20~50 µT），三轴合成后落在几十 µT 量级；
     * 拿块磁铁靠近，读数应显著变大、移开后回去。全 0 或恒定的怪值说明链路有问题。
     * 日志不支持 %f（lib_format 明确不支持），故打印 ×10 的整数，即 0.1µT 单位 */
    BSPLOG(&g_gimbal_log, LOG_LEVEL_INFO, "IST8310 x=%d y=%d z=%d (0.1uT) dt=%dus",
           (int)(data.mag[0] * 10.0f), (int)(data.mag[1] * 10.0f), (int)(data.mag[2] * 10.0f),
           (int)g_ist8310_test_us);
}

#endif /* DRV_IST8310_USED */

// 实例
DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机
RSMOTOR_INSTANCE_DEF(yaw_motor);       // yaw电机（RS05）
static AxisMitLiteInstance pitchup_axis;
static AxisMitLiteInstance yaw_axis;

// 姿态传感器：drvlib_bmi088_kalman（内含 BMI088 通信 + 线性卡尔曼）
/* 姿态、标定修正全在模块内做，app 只取结果：
 *   euler（roll/pitch 由 KF 出，yaw 为世界系积分）、yaw_rate（世界系 yaw 角速度）、
 *   gyro（已按标定修正）、temperature、valid */
BMI088_KALMAN_INSTANCE_DEF(bmi088);

/* ─────────── PC 端标定结果：写死进固件 ───────────
 * static const 落在 .rodata，即"编进 flash 的数值"；模块内不做任何运行期标定。
 * 上场前的流程：
 *   1. 采集：上电静置（陀螺零偏）／自热扫温（温漂斜率）／云台 yaw 轴正反转
 *      （陀螺尺度、非正交）／六位置夹具（acc，可选；注意绕竖直轴转动对 acc 无用）
 *   2. 用 python 拟合出下面的参数
 *   3. 改这里 → 重新编译烧录
 * 换 IMU / 拆装 / 改安装方式后必须重新标定。未标定的项保持零值 = 不修正
 * （scale 填 0 与填 1 等价，都不修正）。
 *
 * 当前只标了陀螺零偏，数值来自静止 55s 实测（本机 IMU 倾斜安装：roll -1.78° pitch +15.39°）：
 *   gx +0.00089  gy -0.00011  gz +0.00169   →  +0.051 / -0.006 / +0.097 °/s
 * ⚠ 这组值当初是在加热器把 IMU 稳到 ~50°C 时测的，而加热器已移除（现在工作在
 *   室温）：gz 的温度系数实测 ≈ 0（r=0.000，而 gz 占 yaw 轴投影的 96%，是决定
 *   yaw 漂移的那根轴），但 gx/gy 约 -0.0023~-0.0025 °/s/°C，25°C 与 50°C 差约
 *   0.06 °/s —— 这组值在室温下未必还准，重标时至少把陀螺零偏重测一遍；
 *   全温区的温漂斜率补进 .gyro.bias_tempco、测量的参考温度填进 temp_ref。 */
static const BMI088_Calib_s s_bmi088_calib = {
    .gyro = {
        /* 2026-09-19 重标：38.2℃ 静止 118.3s@500Hz，脚本
         * drvlib/drvlib_bmi088_calib/bmi088_calib.py（残差法：旧值 + 录到的残差）。
         * 标定前 yaw 漂移 0.0088°/s，残差 z 与它逐位对上，标后应≈0。
         * ⚠ 换 IMU / 拆装 / 改安装方式必须重标重烧；温漂未标（本次温差只有 1℃） */
        .bias = {0.002347f, 0.000987f, 0.001846f},
        // .bias_tempco = {...}, .temp_ref = 25.0f,   // 自热扫温标定后填
        // .scale = {...}, .misalign = {...},         // yaw 轴自转标定后填
    },
    // .acc = { .bias = {...}, .scale = {...}, .misalign = {...} },  // 六位置标定后填
};

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

    // 注册 BMI088 + 卡尔曼（只注册子模块，Config 时配置硬件）
    BSP_ASSERT_APP_CALL(BMI088KalmanRegister(&bmi088));
    /* 配置 BMI088 + 卡尔曼（标定参数见上面的 s_bmi088_calib）
     * 噪声参数取值的依据见 drvlib_bmi088_kalman.h 头注释与下面的注释 */
    BMI088Kalman_Config_s bmi088_cfg = {
        .imu = {
            .spi_e = SPI_BMI088,
            .cs_acc_e = GPIO_BMI088_CS_ACCEL,
            .cs_gyro_e = GPIO_BMI088_CS_GYRO,
            .int_acc_e = GPIO_BMI088_INT_ACCEL,
            .int_gyro_e = GPIO_BMI088_INT_GYRO,
            .daemon_reload = 20,
            .daemon_fault = DAEMON_FAULT_NONE,
            .acc_range = BMI088_ACC_RANGE_3G,     // ±3g，云台不会有大加速度，取最小量程换分辨率
            .acc_bwp = BMI088_ACC_BWP_NORMAL,     // 正常带宽（ODR>400Hz 才必须切 OSR 模式）
            .acc_odr = BMI088_ACC_ODR_400,        // 400Hz：只做低频倾角校正，够用
            .gyro_range = BMI088_GYRO_RANGE_2000, // ±2000dps，留余量避免大机动削顶
            /* ODR=1000Hz/BW=116Hz：任务周期 2ms(500Hz)，2000Hz 的采样用不上，
             * 反而把 250Hz 以上的噪声折返进来；1000Hz 配 116Hz 带宽噪声更低 */
            .gyro_conf = BMI088_GYRO_CONF_1000_116,
            .work_mode = BMI088_MODE_INT,
            .spi_timeout_ms = 10, // SPI IT/DMA 传输超时(ms)
        },

        /* ---- 标定参数：写死进固件的常量 ---- */
        .calib = &s_bmi088_calib,

        /* ---- 线性 KF 噪声 ----
         * r_tilt 起手按"加速度计反算倾角噪声实测 0.14°(rms) → 6e-6 rad²"给，
         * 再放大约 60 倍到 4e-4：留余量给机器人振动与加减速（振动下 acc 反算的
         * 倾角噪声远大于静止值）。q_tilt 默认 1e-5 → 倾角校正时间常数约 0.3s，
         * 安装倾角是静态量，慢一点更抗振动。
         * q_bias = 0：关闭"零偏残差"状态估计。本机零偏几乎只随温度慢漂，
         * 已由固件里写死的标定值 + 温漂斜率覆盖，再开一个残差状态只会和标定
         * 互相打架（KF 拿加速度计当基准，而加速度计对倾角的观测无法区分
         * "倾角变了"和"零偏漂了"，长时间运动后容易把假零偏积进姿态）。
         * 若重标后仍发现 yaw 漂移明显（上电零偏重复性差），可以试着开它。 */
        .q_tilt = 1e-5f,
        .q_bias = 0.0f,
        .r_tilt = 4e-4f,
        .dt_max = 0.01f, // 任务被拖延（app.c 会打 DELAY 日志）时钳位单步积分

        /* ---- 温漂补偿用的温度低通 ----
         * ≈0.2s @500Hz：只留分钟级的温漂，滤掉温度量测的量化噪声与抖动。
         * bias_tempco 还没标（全 0）时本项无影响 */
        .temp_lpf_alpha = 0.01f,

        /* ---- VOFA 调试输出 ----
         * 打开后 drvlib 每帧把 IMU 数据写进 CH1~CH15 并自己发帧（通道表见
         * drvlib_bmi088_kalman.h 头注释），app 这边不用再管 VOFA。
         * ⚠ 与 drv_axis_mit_lite 的 vofa_enable(CH1~CH12)、app_shoot 的调试块
         *   (CH1~CH3) 共用通道号，同一时刻只开一路 */
        .vofa_enable = 1,

    };
    BSP_ASSERT_APP_CALL(BMI088KalmanConfig(&bmi088, &bmi088_cfg));

#ifdef DRV_IST8310_USED
    /* 只注册（纯实例登记、不碰硬件）；真正的 Config 推迟到任务里做，原因见文件顶部 */
    BSP_ASSERT_APP_CALL(IST8310Register(&ist8310));
#endif
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

    /* 姿态更新：读数据 → dt（陀螺时间戳差分）→ 标定修正 → KF → yaw 积分
     * 全在 drvlib_bmi088_kalman 内部（标定值是写死的常量），app 只取结果。
     * valid：姿态可用（已播种）；dt==0 表示本帧没有新陀螺样本、未积分（正常），
     *        此时 euler 仍是上一次的结果，可以直接喂给控制器 */
    BMI088KalmanUpdate(&bmi088);
    BMI088Kalman_Data_t imu_data = BMI088KalmanGetData(&bmi088);

    /* ---- yaw 轴反馈：取自 IMU（世界系），不用电机编码器 ----
     * IMU 刚性固定在云台底座（yaw 轴输出、两个 pitch 关节的上游）：
     *   ① 两个 pitch 轴怎么转都带不动 IMU，只有 yaw 轴转动会改变它的姿态；
     *   ② 因此它报告的 roll/pitch 就是恒定的安装倾角（-1.78° / +15.39°），
     *      euler.yaw 就是 yaw 轴的世界系航向，与视觉下发的世界系 yaw 同一坐标系。
     * 位置用 euler.yaw：wrap 到 (-π, π]，与 yaw 轴配置里的 error_normalize_enable=1
     *   对齐（位置环按最短路径算误差，边界处不会跳）。
     * 速度用 drvlib 给的 yaw_rate（世界系 ψ̇，由机体系陀螺仪经欧拉角运动学精确换算），
     *   不能直接取 gyro.z：安装倾角 15.39° 使 cosθ·cosφ≈0.963，gyro.z 只有真实航向
     *   角速度的 96%，另外还有一部分落在 gyro.x 上。
     * 为什么不用编码器：编码器给的是"相对底盘"的关节角，底盘自转/被推动/回差时
     * 都不等于云台的真实指向；要控世界系航向就得拿世界系的量来控。 */
    AxisLiteState_s yaw_state = {
        .position = imu_data.euler.yaw,
        .speed = imu_data.yaw_rate,
        .torque = yaw_mdata.torque,
    };

    /* VOFA 调试输出已移到 drvlib 里：app_gimbal 的 bmi088_cfg 里把
     *   .vofa_enable = 1
     * 打开，drvlib 每帧自己把 IMU 数据写进通道并 VofaSend（通道表在
     * drvlib_bmi088_kalman.h 头注释"VOFA 调试输出"一节，CH1~CH15）。
     * 验证标定：CH13-15 上电后应恒等于 s_bmi088_calib 里的数值（tempco 未标定时）；
     *   看 CH12（温度）与 CH4-6 均值漂移可判断是否还需要温漂标定；
     *   yaw 漂移速率看 CH3 长时间趋势，转动 yaw 时 CH11 应约等于 d(CH3)/dt，
     *   且 CH1-2 基本不动。
     * 注意：帧由 drvlib 发出，app 不要再自己调 VofaSend（会多发一帧）；
     *       而通道号是全局的 —— drv_axis_mit_lite 的 vofa_enable 占用 CH1~CH12、
     *       app_shoot.c 的调试块占用 CH1~CH3（那三路眼下没人发帧，属死写），
     *       要同时看就得把它们错开或用别的通道号 */

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

    /* 回传云台反馈给 cmd（规划器需要当前位置/速度；pitch_down 供视觉回传下pitch位姿角）。
     * yaw 与 yaw_vel 回传的是 IMU 世界系量：必须与上面 yaw 轴控制用的反馈同源，
     * 否则 cmd 的规划器会拿"相对底盘"的锚点去规划"世界系"的目标，每帧都差一个底盘航向。
     * 同时视觉下发的 yaw 本就是世界系（见 app_cmd send_gimbal 的视觉分支），回传也对齐。
     * yaw_motor_* 另外回传编码器关节角（世界系量丢掉了"云台相对底盘"这一信息）：
     *   ⚠ cmd 侧 chassis_w_from_mode 的底盘跟随 w = kp*wrap(-yaw_position) 要的是相对角，
     *     现在拿的是世界系航向，normal/gyro 下会持续自转。这两个字段就是给它预留的，
     *     cmd 侧切过来后，底盘跟随改用 yaw_motor_position/yaw_motor_vel 即可。 */
    gimbal2cmd_data_t gimbal2cmd_data = {
        .pitch_position = pitchup_mdata.position,
        .pitch_vel = pitchup_mdata.speed,
        .yaw_position = imu_data.euler.yaw,
        .yaw_vel = imu_data.yaw_rate,
        .pitch_down_position = pitchdown_mdata.position,
        .yaw_motor_position = (float)yaw_mdata.position,
        .yaw_motor_vel = yaw_mdata.speed,
    };
    xQueueOverwrite(gimbal2cmd_queue_handle, &gimbal2cmd_data);

#ifdef DRV_IST8310_USED
    IST8310_TestPolling(); // 临时：IST8310 阻塞读 bring-up 测试
#endif
}
