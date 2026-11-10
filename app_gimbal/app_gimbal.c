#include "app_gimbal.h"
#include "app_cfg.h"
#include "app.h"
#include "app_cmd_gimbal.h" // gimbal2cmd / cmd2gimbal 队列句柄与数据结构
#include "robot_def.h"
//
#include "drv_dmmotor.h"
#include "drv_rsmotor.h"
#include "lib_motor.h"
#include "drv_vofa.h"
#include "lib_axis_pid_lite.h"
#include "drvlib_bmi088_kalman.h"
#include "lib_fsm_table.h" // 表驱动状态机（云台状态机）
//
#include "bsp_sys_status.h"
//
#include <stdbool.h> // 状态机守卫

/* VOFA 自发送开关：axis 模块（lib_axis_pid_lite）只写通道、不自己发帧，
 * 整定云台轴时由 AppGimbalRun 每拍发一帧（本任务 500 Hz）。
 * ⚠ 同一时刻只能有一个发送者：若改用 drvlib_bmi088_kalman 的 vofa_enable（它自己发帧），
 *   或打开 app_shoot 的调试块（SHOOT_VOFA_ENABLE=1），必须把这里置 0 ——
 *   否则通道值互相覆盖，而且每拍会发两帧。
 * ⚠ 与 axis 的 vofa_enable 配套：本开关=1 但没有任何 axis 打开 vofa_enable 时，
 *   发出去的是残留通道值。 */
#define GIMBAL_VOFA_SEND 1

// 实例
DRV_DMMOTOR_INSTANCE_DEF(pitchdown_motor); // 下pitch电机
DRV_DMMOTOR_INSTANCE_DEF(pitchup_motor);   // 上pitch电机
DRV_RSMOTOR_INSTANCE_DEF(yaw_motor);       // yaw电机（RS05）
/* 三轴控制核：开环透传，只借用 lib_motor 的反馈后处理（多圈/方向/低通） */
LIB_MOTOR_INSTANCE_DEF(pitchdown_ctrl);
LIB_MOTOR_INSTANCE_DEF(pitchup_ctrl);
LIB_MOTOR_INSTANCE_DEF(yaw_ctrl);
static AxisPidLiteInstance pitchup_axis;
static AxisPidLiteInstance pitchdown_axis;
static AxisPidLiteInstance yaw_axis;

/* 掉线恢复：DM/RS 通信丢失后电机会自行退出 MIT 模式。drv 层刻意不持有使能状态
 * （不知道该不该重发），由 app 在 daemon 离线回调里重发使能 —— 云台三轴上电后
 * 始终需要扭矩（失能态只把输出置 0，不发停机命令），故重发策略恒为"使能"。 */
static void GimbalDMMotorReenable(void *owner)
{
    DrvDMMotorSendCmd((DrvDMMotor_s *)owner, DRV_DM_CMD_MOTOR_MODE);
}

static void GimbalRSMotorReenable(void *owner)
{
    DrvRSMotorSendCmd((DrvRSMotor_s *)owner, DRV_RS_CMD_MOTOR_MODE);
}

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
    .gyro =
        {
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
static uint64_t gimbal_enable_last_ms = 0; // 上次周期重发使能的时刻 (ms)

/*============================================
 *        云台状态机（lib_fsm_table）
 *   状态 = 机构实际姿态，不是 cmd 下发的档位：
 *     失能 ←→ { 立起中 → 直立 } / { 倒下中 → 倒下 }
 *   驱动量有两个：
 *     ① cmd 下发的 robot_mode → 目标姿态：stop=失能、normal/gyro=直立、hole=倒下
 *        （小陀螺在云台侧与普通同姿态：差异是底盘 w，由 cmd 算，云台不参与）
 *     ② 下 pitch 位置 → 实际姿态：拿 pitchdown_position_max/min 两端点判"到位"
 *   产出：本状态下哪几轴由 cmd / 姿态目标驱动 ——
 *     失能          ：三轴输出 0（电机保持使能，仍发零力矩帧）
 *     立起中        ：下 pitch = 重力前馈 + 速度环（朝 pitchdown_position_max），
 *                     上 pitch 跟随 cmd 设定值
 *     直立          ：下 pitch = 重力前馈 + 固定偏置 + 上 pitch 单向耦合，
 *                     上 pitch 跟随 cmd 设定值
 *     倒下中        ：下 pitch = 重力前馈 + 速度环（朝 pitchdown_position_min）、
 *                     上 pitch 驶向相对角 0；但**要先等 yaw 相对底盘对正**，
 *                     未对正时下 pitch 速度给定为 0（起阻尼作用）、上 pitch 保持
 *     倒下          ：下 pitch 输出 0（关闭重力前馈，靠自重/限位停住），上 pitch 保持在 0
 *   yaw 在除失能外的所有状态继续控世界系航向。
 *   ⚠ 下 pitch 没有位置环：上升/下降阶段的速度给定是常量 ±GIMBAL_POSTURE_SLEW_RATE。
 *     真正的迟滞、卡死超时等以后需要时再加状态动作。
 *   ⚠ 自瞄(visual) 不是云台状态：云台看不见视觉，只是跟随 cmd 算好的设定值（cmd 把视觉
 *     规划写进 pitch/yaw 的 x/v/a）。若将来自瞄要让云台本身改行为（增益/限幅/别的轴
 *     策略），再给 robot_mode 加值或给云台单独发标志。
 *============================================*/

/* 到位判据阈值：下 pitch 距端点的距离 (rad)，立起/倒下各自独立。
 *   立起端：保持较紧，避免没抬到位就报直立；
 *   倒下端：放宽松。倒下由速度环驱动、到点后只剩重力前馈（没有位置环），
 *           实际常停在 pitchdown_position_min 上方，阈值太紧会一直卡在 falling。 */
#define GIMBAL_POSTURE_STAND_EPS 0.10f
#define GIMBAL_POSTURE_LYE_EPS 0.20f

/* 姿态过渡速度 (rad/s)：
 *   下 pitch —— 直接作为速度环给定（上升 +、下降 -、未许可下降时为 0）；
 *   上 pitch —— 仍作为位置设定值的限速和前馈速度。
 * 取值兼顾“有力但不砸”，并给下 pitch 明确的下压/抬升速度。 */
#define GIMBAL_POSTURE_SLEW_RATE 2.0f

/* “yaw 轴对准”判据：云台相对底盘前进方向的夹角 θ = wrap(yaw 编码器 - chassis_gimbal_offset)
 * 进入死区，且相对角速度足够小，才允许开始主动倒下。
 * 机械原因：倒下后云台沿底盘纵向折叠，yaw 没和底盘对正时折叠会别住/扫到车体。
 * hole 档底盘本来就在跟随把 θ 拉回 0（底盘侧 ChassisWCommand 自己读共享 CAN1 的
 * RS05 得到实测 θ），所以这个条件会自然收敛。
 * TODO 实车标定：以折叠不干涉为准；若希望“完全静止才倒下”，把速度阈值再改小。 */
#define GIMBAL_YAW_ALIGN_EPS DEG_TO_RAD(6.0f)
#define GIMBAL_YAW_ALIGN_SPEED DEG_TO_RAD(10.0f)

/* 下 pitch 控制律：只用 axis 模块的重力前馈，其余项都在 app 层按状态叠加：
 *   恒发   ：重力前馈 gravity*cos(pos)（已实测准确，gravity=1.5）；
 *   上升/下降：+ 速度闭环；
 *   最高点锁定（stand）：+ 固定偏置 + 上 pitch 单向耦合；
 *   倒下中（falling）：重力前馈 + 速度闭环；
 *   倒下（lying）：输出 0，关闭重力前馈，靠自重和机械限位停住。 */
#define GIMBAL_PITCHDOWN_GRAVITY 1.5f
#define GIMBAL_PITCHDOWN_TORQUE_BIAS 2.0f // 最高点锁定时叠加的固定偏置（按实车标定）
#define PITCH_UP2DOWN_KP 0.0f

/* 速度环增益 (Nm/(rad/s))，仅上升/下降阶段工作。
 * 旧律速度项 kd*(v_ref-speed) 的 kd 约 1，先从 1.0 起调；过大会在换向/到位时振荡。 */
#define GIMBAL_PITCHDOWN_VEL_KV 0.5f

/* 周期重发使能 (ms)。
 * 只在初始化发一次 0xFC 时：若 STM 先上电、电机后上电（或 RS05 上电后还在 CALI，
 * 期间不接受 MIT 命令），那一帧就丢了，电机一直停在未使能。这里每 100ms 无条件重发，
 * 任何时刻上电的电机最多 100ms 后进入 MIT 模式。离线回调仍保留，负责断线期间的快速重试。 */
#define GIMBAL_ENABLE_RESEND_MS 100u

/* axis 模块只保留重力前馈（step1）。速度环/偏置/耦合都在 app 层另加；
 * 不要切到 NORMAL，否则模块内部的位置环会和这套律叠加。 */
#define GIMBAL_PITCHDOWN_AXIS_STAGE AXIS_LITE_STAGE_FIXED_TORQUE

/* 姿态过渡用位置设定值（上 pitch 限速后喂给 axis 模块；下 pitch 仅作状态锚点）
 * 与状态边沿检测 */
static float pitchdown_posture_ref = 0.0f;
static float pitchup_posture_ref = 0.0f;
static gimbal_state_e gimbal_state_last = gimbal_state_disable_e;
/* yaw 对准后锁存：一旦开始倒下就允许走完，不因中途 θ 抖动又停在半路 */
static uint8_t gimbal_fall_permitted = 0;

/* 把 *cur 每拍朝 target 最多挪 max_delta，返回实际步长（秒化后作为 ref.speed 前馈） */
static inline float GimbalSlewStep(float *cur, float target, float max_delta)
{
    float step = Lib_Math_Clamp(target - *cur, -max_delta, max_delta);
    *cur += step;
    return step;
}

typedef enum : uint8_t
{
    gimbal_ev_update_e = 0, // cmd 下发模式 或 机构实际姿态 变了
} gimbal_event_e;

/* 由下 pitch 位置解出的实际姿态：机构行程的两端 + 中间 */
typedef enum : uint8_t
{
    gimbal_posture_unknown_e = 0, // 在两端之间（运动中）
    gimbal_posture_stand_e = 1,   // 已抬到直立端
    gimbal_posture_lye_e = 2,     // 已落到倒下端
} gimbal_posture_e;

typedef struct
{
    robot_mode mode;               // 本拍 cmd 下发的模式（目标姿态来源）
    robot_mode mode_last;          // 上一拍模式（边沿检测）
    gimbal_posture_e posture;      // 本拍由位置解出的实际姿态（守卫读它）
    gimbal_posture_e posture_last; // 上一拍姿态（边沿检测）
} GimbalFsmCtx_s;

static GimbalFsmCtx_s gimbal_fsm_ctx;
static LibFsmTableInstance_s gimbal_fsm;

/*---------------- 守卫 ----------------*/
/* 守卫必须无副作用且幂等：同 from/event 的多条规则会依次求值（见 lib_fsm_table.h）。
 * 下面五个条件互斥，故表内先后只决定"失能优先"这一条语义。 */
static uint8_t GimbalFsmTargetIsStand(const GimbalFsmCtx_s *c)
{
    return (c->mode == robot_mode_normal) || (c->mode == robot_mode_gyro);
}
static bool GimbalFsmGuardDisable(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((GimbalFsmCtx_s *)ctx)->mode == robot_mode_stop;
}
static bool GimbalFsmGuardNeedRise(void *ctx, const void *event_data)
{
    (void)event_data;
    GimbalFsmCtx_s *c = (GimbalFsmCtx_s *)ctx;
    return GimbalFsmTargetIsStand(c) && (c->posture != gimbal_posture_stand_e);
}
static bool GimbalFsmGuardStandReached(void *ctx, const void *event_data)
{
    (void)event_data;
    GimbalFsmCtx_s *c = (GimbalFsmCtx_s *)ctx;
    return GimbalFsmTargetIsStand(c) && (c->posture == gimbal_posture_stand_e);
}
static bool GimbalFsmGuardNeedFall(void *ctx, const void *event_data)
{
    (void)event_data;
    GimbalFsmCtx_s *c = (GimbalFsmCtx_s *)ctx;
    return (c->mode == robot_mode_hole) && (c->posture != gimbal_posture_lye_e);
}
static bool GimbalFsmGuardLyeReached(void *ctx, const void *event_data)
{
    (void)event_data;
    GimbalFsmCtx_s *c = (GimbalFsmCtx_s *)ctx;
    return (c->mode == robot_mode_hole) && (c->posture == gimbal_posture_lye_e);
}

/*---------------- 转移表 ----------------*/
/* 源状态一律通配：五条守卫已把 (目标姿态 × 实际姿态) 完全划分，落点唯一；
 * 表顺序 = 优先级，失能排最前（cmd 说 stop 时无论机构在哪都直接进失能）。 */
static const LibFsmTableTransition_s gimbal_fsm_table[] = {
    {LIB_FSM_TABLE_ANY_STATE, gimbal_ev_update_e, gimbal_state_disable_e, GimbalFsmGuardDisable, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, gimbal_ev_update_e, gimbal_state_rising_e, GimbalFsmGuardNeedRise, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, gimbal_ev_update_e, gimbal_state_stand_e, GimbalFsmGuardStandReached, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, gimbal_ev_update_e, gimbal_state_falling_e, GimbalFsmGuardNeedFall, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, gimbal_ev_update_e, gimbal_state_lying_e, GimbalFsmGuardLyeReached, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
};
LIB_FSM_TABLE_CHECK_TABLE(gimbal_fsm_table);

/*---------------- 每拍推进 ----------------*/
/* 姿态是 (模式, 位置) 的函数，但换状态的两条边（cmd 换档、机构到位）各自只在**变化**时
 * 派发：否则同值也走一次外部转移（退出+进入各跑一遍），将来挂上状态动作就会每拍重跑。 */
static void GimbalFsmUpdate(robot_mode mode, float pitchdown_pos)
{
    gimbal_fsm_ctx.mode = mode;
    gimbal_fsm_ctx.posture = gimbal_posture_unknown_e;
    if (pitchdown_pos >= pitchdown_position_max - GIMBAL_POSTURE_STAND_EPS)
    {
        gimbal_fsm_ctx.posture = gimbal_posture_stand_e;
    }
    else if (pitchdown_pos <= pitchdown_position_min + GIMBAL_POSTURE_LYE_EPS)
    {
        gimbal_fsm_ctx.posture = gimbal_posture_lye_e;
    }
    if ((mode != gimbal_fsm_ctx.mode_last) || (gimbal_fsm_ctx.posture != gimbal_fsm_ctx.posture_last))
    {
        LibFsmTableDispatch(&gimbal_fsm, gimbal_ev_update_e, NULL);
        gimbal_fsm_ctx.mode_last = mode;
        gimbal_fsm_ctx.posture_last = gimbal_fsm_ctx.posture;
    }
}

/* 外部函数 */
void AppGimbalInit(void)
{
    // 注册 CAN 实例（下pitch、上pitch 共用 CAN_2）
    BSP_ASSERT_APP_CALL(DrvDMMotorRegister(&pitchdown_motor));
    BSP_ASSERT_APP_CALL(DrvDMMotorRegister(&pitchup_motor));
    BSP_ASSERT_APP_CALL(DrvRSMotorRegister(&yaw_motor));

    /* drv 层只做协议映射：量程必须与上位机一致（DM 调试助手 / 灵足上位机）。
     * 方向、滤波、多圈、开环透传全在下面的 lib_motor 配置里。 */
    // 配置下pitch
    DrvDMMotorConfig_s pitchdown_cfg = {
        .can_e = CAN_2,
        .can_id = 0x001,
        .master_id = 0x011,
        .pos_max = (float)M_PI, // DM 位置 ±π
        .vel_range = 30.0f,     // DM 速度 ±30 rad/s
        .t_range = 10.0f,       // DM 扭矩 ±10 Nm
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvDMMotorConfig(&pitchdown_motor, &pitchdown_cfg));

    // 配置上pitch
    DrvDMMotorConfig_s pitchup_cfg = {
        .can_e = CAN_2,
        .can_id = 0x002,
        .master_id = 0x012,
        .pos_max = (float)M_PI,
        .vel_range = 30.0f,
        .t_range = 10.0f,
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvDMMotorConfig(&pitchup_motor, &pitchup_cfg));

    // 配置yaw（RS05，量程需与灵足上位机一致：位置±12.57rad/速度±50rad/s/力矩±5.5Nm）
    DrvRSMotorConfig_s yaw_cfg = {
        .can_e = CAN_1,
        .can_id = 0x01,
        .master_id = 0xfd,
        .pos_max = 12.57f,  // RS05 默认量程
        .vel_range = 50.0f, // RS05 默认量程
        .t_range = 5.5f,    // RS05 默认量程
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvRSMotorConfig(&yaw_motor, &yaw_cfg));

    /* 三轴控制核（开环）：app 侧用 axis_lite 算力矩，lib_motor 只做反馈后处理，
     * 落点与旧 drv_motor 相同：
     *   - pitch：旧 LIMITED(0,0) 实际不限幅 → CONTINUOUS；span=2π（DM 协议 ±π）
     *   - yaw  ：WRAP ±π、span=0（无减速箱，直接用单圈绝对位置，不做多圈累加）；
     *            feedback/motor 均 REVERSE（镜像后同步翻）
     *   - 速度低通 RC 沿用旧整定；转矩不滤波 */
    LibMotor_Config_s pitchdown_ctrl_cfg = {
        .loop_type = LIB_MOTOR_LOOP_OPEN,
        .motor_direction = LIB_MOTOR_DIRECTION_NORMAL,    // 输出方向
        .feedback_direction = LIB_MOTOR_DIRECTION_NORMAL, // 反馈方向
        .position_mode = LIB_MOTOR_POS_CONTINUOUS,
        .position_span = 2.0f * (float)M_PI,
        .position_offset = 0.0f,
        .lpf_enable = LIB_MOTOR_LPF_ENABLE,
        .lpf_rc = 0.1f,
        .current_lpf_rc = 0.0f,
        .dt_max = 0.01f,
        .vofa_enable = 0,
    };
    LibMotor_Config_s pitchup_ctrl_cfg = pitchdown_ctrl_cfg;
    pitchup_ctrl_cfg.lpf_rc = 0.004f; // 截止~40Hz, kp=40时ωn=11.3Hz总滞后140°裕度30° (电机8.6ms延迟限定了高kp必振荡)
    /* 静止时编码器在两相邻计数间抖动，DM 内部测速给出每拍交替的 ±0.04 rad/s（fs/2）。
     * 一阶 RC 在该频率只压一半，再串 2 点滑动平均把 Nyquist 分量消掉；
     * 低频只多 1 ms 延迟（0.9 Hz 相位 -0.36°）。 */
    pitchup_ctrl_cfg.speed_ma_enable = 1;

    LibMotor_Config_s yaw_ctrl_cfg = {
        .loop_type = LIB_MOTOR_LOOP_OPEN,
        .motor_direction = LIB_MOTOR_DIRECTION_REVERSE,    // 输出方向：镜像后 正力矩→逆时针（与反馈同步翻，闭环稳定）
        .feedback_direction = LIB_MOTOR_DIRECTION_REVERSE, // 反馈方向：镜像后 逆时针→正角度（原编码器逆时针为负）
        .position_mode = LIB_MOTOR_POS_WRAP,
        .angle_limit_min = -(float)M_PI,
        .angle_limit_max = (float)M_PI,
        .position_span = 0.0f, // 无减速箱：编码器单圈位置即绝对 wrap 角，不做软件多圈累加（避免圈数多后累计误差）
        .position_offset = 0.0f,
        .lpf_enable = LIB_MOTOR_LPF_ENABLE,
        .lpf_rc = 0.004f,
        .current_lpf_rc = 0.0f,
        .dt_max = 0.01f,
        .vofa_enable = 0,
    };

    BSP_ASSERT_APP_CALL(LibMotorRegister(&pitchdown_ctrl));
    BSP_ASSERT_APP_CALL(LibMotorRegister(&pitchup_ctrl));
    BSP_ASSERT_APP_CALL(LibMotorRegister(&yaw_ctrl));
    BSP_ASSERT_APP_CALL(LibMotorConfig(&pitchdown_ctrl, &pitchdown_ctrl_cfg));
    BSP_ASSERT_APP_CALL(LibMotorConfig(&pitchup_ctrl, &pitchup_ctrl_cfg));
    BSP_ASSERT_APP_CALL(LibMotorConfig(&yaw_ctrl, &yaw_ctrl_cfg));

    LibMotorEnable(&pitchdown_ctrl);
    LibMotorEnable(&pitchup_ctrl);
    LibMotorEnable(&yaw_ctrl);

    /* 协议层使能 + 掉线重使能（drv 不持有使能状态，策略归 app）。
     * 这里先发一次；AppGimbalRun 里还会每 GIMBAL_ENABLE_RESEND_MS 无条件重发，
     * 覆盖"STM 先上电、电机后上电"和 RS05 上电后 CALI 期间丢命令的情况。 */
    DrvDMMotorSendCmd(&pitchdown_motor, DRV_DM_CMD_MOTOR_MODE);
    DrvDMMotorSendCmd(&pitchup_motor, DRV_DM_CMD_MOTOR_MODE);
    DrvRSMotorSendCmd(&yaw_motor, DRV_RS_CMD_MOTOR_MODE);
    DaemonConfig(pitchdown_motor.daemon,
                 &(Daemon_Config_s){.reload_count = 100, .fault_action = DAEMON_FAULT_NONE, .callback = GimbalDMMotorReenable, .owner_id = &pitchdown_motor});
    DaemonConfig(pitchup_motor.daemon,
                 &(Daemon_Config_s){.reload_count = 100, .fault_action = DAEMON_FAULT_NONE, .callback = GimbalDMMotorReenable, .owner_id = &pitchup_motor});
    DaemonConfig(yaw_motor.daemon,
                 &(Daemon_Config_s){.reload_count = 100, .fault_action = DAEMON_FAULT_NONE, .callback = GimbalRSMotorReenable, .owner_id = &yaw_motor});

    AxisPidLite_Init_Config_s pitchup_axis_cfg = {
        .stage = AXIS_LITE_STAGE_NORMAL, // 控制阶段
        .delay_ms = 5000,                // 延时时间 (ms)
        // .vofa_enable = AXIS_PID_LITE_VOFA_BASE | AXIS_PID_LITE_VOFA_VEL_PID,
        // .vofa_base_ch = 1,
        // .vofa_vel_pid_base_ch = 13, // 速度环内部变量起始通道，占 base ~ base+5，0 按 1 处理
        .params =
            {
                .gravity = 0.389f, // 重力前馈系数（标定 0.28→0.30）
                .gear_ratio = 1,
                .inertia = 0.012f, // kg·m²（标定 0.012→0.008，前馈过大导致振幅放大）
                .friction_coulomb_pos = 0.0f,
                .friction_coulomb_neg = 0.0f,
                .friction_viscous_pos = 0.0f,
                .friction_viscous_neg = 0.0f,
            }, // 轴参数
        .sine_params =
            {
                .amplitude = 0.15f,
                .freq = 1.5f,
            }, // 正弦参数
        .chirp_params =
            {
                .amplitude_start = 0.1f,
                .amplitude_end = 3,
                .duration = 15,
                .start_freq = 1,
                .end_freq = 8,
            }, // 扫频参数
        .multi_sine_params =
            {
                .amplitude = 0.1f,
                .duration = 1,
                .num_freqs = 10,
            }, // 多正弦叠加参数
        /* 串级 PID 与 MIT 版等效：
         *   τ = vel_kp*[(pos_kp*(ref-pos) + ref_vel) - speed] + ff
         *     = (pos_kp*vel_kp)*(ref-pos) + vel_kp*(ref_vel-speed) + ff
         * 原 MIT 整定 kp=10 / kd=0.4，取 vel_kp=0.4、pos_kp=25 即等效（刚度 25*0.4=10）。
         * 电机延迟 8.6ms 限定 kp 上限：kp=80 必振荡(16Hz位置环极限环)，
         * kp=40 总滞后140°裕度30°。两环只开 P（ki/kd=0），与 MIT 的 PD 严格等效。 */
        .pos_pid =
            {
                .kp = 35,                   // 比例系数
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
        /* 速度环整定依据（ig/vofa+.csv，TUNE 2 Hz 正弦，幅值 0.15 rad）：
         *   实测跟踪误差 0.290 rad/s 峰峰值（增益 0.87、相位 +5°）；
         *   用实测响应反推的等效被控对象约为 1/(J s + b)，J≈0.007、b≈0.10，
         *   叠加约 14 ms 延迟后，P 环 kp=0.55 在 2 Hz 的预测误差为 0.287，与实测吻合。
         *   - kp 保持 0.55：增益穿越 ≈77 rad/s(12.3 Hz)，计入延迟后相位裕度约 38°，
         *     已接近该轴可用上限（kp=0.6 时裕度降到约 32°），不再加大。
         *   - ki=2：积分转折 ki/kp≈3.6 rad/s(0.58 Hz)。2 Hz 处积分几乎不改变误差，
         *     但 0.5 Hz 跟踪误差可降约 26%，并能把常值摩擦/重力残差造成的稳态误差压到 0。
         *   - 积分限幅 0.25 Nm：覆盖最大速度下约 0.19 Nm 的等效阻尼力矩 + 摩擦余量，
         *     同时避免大误差时积分在重力前馈上叠出过大偏置。
         *   - 变速积分：|e|≤0.2 rad/s 全积分（小误差/稳态用），0.2~0.8 rad/s 线性减弱，
         *     >0.8 rad/s 停止积分，防止换向/视觉跳变等大误差时积分饱和造成过冲。 */
        .vel_pid =
            {
                .kp = 0.55f,                     // 比例系数 Nm/(rad/s)
                .ki = 2.0f,                      // 积分系数 Nm/(rad/s·s)
                .kd = 0,                         // 微分系数
                .integral_limit = 0.25f,         // 积分限幅阈值 (Nm)
                .coef_a = 0.6f,                  // 变速积分参数 A (rad/s)：0.2~0.8 线性减弱
                .coef_b = 0.2f,                  // 变速积分参数 B (rad/s)：|e|≤0.2 全积分
                .d_lpf_rc = 0,                   // 微分滤波时间常数 RC (0 = 禁用)
                .out_lpf_rc = 0,                 // 输出滤波时间常数 RC (0 = 禁用)
                .deadband = 0,                   // 死区范围 (0 = 禁用)
                .error_normalize_range = 0,      // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
                .out_max = 0,                    // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
                .out_min = 0,                    // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
                .config_mask =                   // 功能配置掩码
                PID_ENABLE_INTEGRAL_LIMIT |      // 启用积分限幅
                PID_ENABLE_TRAPEZOID_INTEGRAL |  // 启用梯形积分
                PID_ENABLE_CHANGING_INTEGRATION, // 启用变速积分
            },
    };
    BSP_ASSERT_APP_CALL(AxisPidLiteInit(&pitchup_axis, &pitchup_axis_cfg));

    /* 下 pitch 轴：只用 FIXED_TORQUE 阶段的重力前馈 gravity*cos(pos)，
     * 模块内部的位置/速度闭环不参与；速度环、最高点偏置、上 pitch 耦合都在
     * AppGimbalRun 里按状态叠加（见 GIMBAL_PITCHDOWN_* 宏）。
     * gravity=1.5 已实测准确；双环 PID 在 FIXED_TORQUE 下不生效，仅保留配置。 */
    AxisPidLite_Init_Config_s pitchdown_axis_cfg = {
        .stage = GIMBAL_PITCHDOWN_AXIS_STAGE,
        .delay_ms = 5000,
        .params =
            {
                .gravity = GIMBAL_PITCHDOWN_GRAVITY,
                .gear_ratio = 1,
                .inertia = 0.008f,
                .friction_coulomb_pos = 0.0f,
                .friction_coulomb_neg = 0.0f,
                .friction_viscous_pos = 0.0f,
                .friction_viscous_neg = 0.0f,
            },
        .sine_params = {.amplitude = 0.2f, .freq = 2},
        .chirp_params = {.amplitude_start = 0.1f, .amplitude_end = 3, .duration = 15, .start_freq = 1, .end_freq = 8},
        .multi_sine_params = {.amplitude = 0.1f, .duration = 1, .num_freqs = 10},
        /* FIXED_TORQUE 阶段 PID 不参与闭环，仅保留原 MIT 增益的等效占位：
         * kp=4 / kd=2 ⇒ vel_kp=2、pos_kp=4/2=2。 */
        .pos_pid = {.kp = 2.0f},
        .vel_pid = {.kp = 2.0f},
    };
    BSP_ASSERT_APP_CALL(AxisPidLiteInit(&pitchdown_axis, &pitchdown_axis_cfg));

    AxisPidLite_Init_Config_s yaw_axis_cfg = {
        .stage = AXIS_LITE_STAGE_NORMAL, // 控制阶段
        .delay_ms = 5000,                // 延时时间 (ms)
        .vofa_enable = AXIS_PID_LITE_VOFA_BASE | AXIS_PID_LITE_VOFA_VEL_PID,
        .vofa_base_ch = 1,
        .vofa_vel_pid_base_ch = 13, // 速度环内部变量起始通道，占 base ~ base+5，0 按 1 处理
        .params =
            {
                .gravity = 0.0f,
                .gear_ratio = 1,
                .inertia = 0.015f,
                .friction_coulomb_pos = 0.0f,
                .friction_coulomb_neg = 0.0f,
                .friction_viscous_pos = 0.0f,
                .friction_viscous_neg = 0.0f,
            }, // 轴参数
        .sine_params =
            {
                .amplitude = 0.2f,
                .freq = 2,
            }, // 正弦参数
        .chirp_params =
            {
                .amplitude_start = 0.1f,
                .amplitude_end = 3,
                .duration = 15,
                .start_freq = 1,
                .end_freq = 8,
            }, // 扫频参数
        .multi_sine_params =
            {
                .amplitude = 0.1f,
                .duration = 1,
                .num_freqs = 10,
            }, // 多正弦叠加参数
        /* 等效原 MIT 整定 kp=5 / kd=0.8：vel_kp=0.8、pos_kp=5/0.8=6.25。
         * 两环只开 P（ki/kd=0），与 MIT 的 PD 严格等效。 */
        .pos_pid =
            {
                .kp = 12, // 位置环 (rad/s)/rad，输出为速度设定值；等效刚度 6.25*0.8=5
            },
        /* 速度环整定依据（ig/0.8_0.csv / ig/0.8_6.25.csv，TUNE 2 Hz 正弦，幅值 0.2 rad）：
         *   pos_kp=0 时实测 T=speed/ref_vel=0.965(-0.31 dB)、相位 +1.8°；
         *   纯 P 已把 2 Hz 基频误差压到 4.6%。用指令力矩 I18（I3 反馈有约 0.53 倍
         *   缩放和 ~31 ms 延迟，只适合看趋势）反推被控对象 1/(J s + b)：
         *   J≈0.0138、b≈0.035；前馈 J=0.015 基本匹配，预测 T=0.964∠1.5°，
         *   与实测吻合，说明 2 Hz 误差主要来自残余阻尼。
         *   - kp 保持 0.8：增益穿越 kp/J≈58 rad/s(9.2 Hz)，被控对象近似一阶，
         *     相位裕度约 90°，当前没有振荡迹象。先不加 kp，避免把 6/10 Hz
         *     机械非线性（3 次/5 次谐波）也一起放大。
         *   - ki=2：取 kp*b/J=0.8*0.035/0.0138≈2.0，使积分零点 ki/kp≈2.5 rad/s
         *     正好抵消被控对象极点 b/J≈2.5 rad/s(0.4 Hz)。速度环因此近似一阶，
         *     低频/常值摩擦误差积分到 0；2 Hz 处积分只改变误差相位，误差幅值
         *     4.47%→4.56%（可忽略），不会恶化正弦跟踪。
         *   - 积分限幅 0.2 Nm：yaw 无重力，2 Hz 最大速度下等效阻尼力矩约
         *     0.035*2.4≈0.084 Nm，0.2 给静摩擦留余量；若慢速保持测试里 i_out
         *     长时间饱和且仍有残差，再放宽到 0.25~0.3。
         *   - 变速积分：|e|≤0.1 rad/s 全积分（稳态用），0.1~0.4 rad/s 线性减弱，
         *     >0.4 rad/s 停止积分，防止启动/阶跃/视觉跳变时积分饱和造成过冲。
         *   - 梯形积分：500 Hz 下减小采样积分偏差。 */
        .vel_pid =
            {
                .kp = 1.2f,                      // 比例系数 Nm/(rad/s)
                .ki = 2.0f,                      // 积分系数 Nm/(rad/s·s)
                .kd = 0,                         // 微分系数
                .integral_limit = 0.2f,          // 积分限幅阈值 (Nm)
                .coef_a = 0.3f,                  // 变速积分参数 A (rad/s)：0.1~0.4 线性减弱
                .coef_b = 0.1f,                  // 变速积分参数 B (rad/s)：|e|≤0.1 全积分
                .d_lpf_rc = 0,                   // 微分滤波时间常数 RC (0 = 禁用)
                .out_lpf_rc = 0,                 // 输出滤波时间常数 RC (0 = 禁用)
                .deadband = 0,                   // 死区范围 (0 = 禁用)
                .error_normalize_range = 0,      // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
                .out_max = 0,                    // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
                .out_min = 0,                    // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
                .config_mask =                   // 功能配置掩码
                PID_ENABLE_INTEGRAL_LIMIT |      // 启用积分限幅
                PID_ENABLE_TRAPEZOID_INTEGRAL |  // 启用梯形积分
                PID_ENABLE_CHANGING_INTEGRATION, // 启用变速积分
            },
        // yaw 是 WRAP 环绕轴（±π 归一化）：误差需取最短路径，否则边界处跳变
        .error_normalize_range = 2.0f * M_PI, // 误差 wrap 到 [-π, π)
        .error_normalize_enable = 1,          // 启用环绕误差归一化
        // TUNE 正弦参考默认以延时结束时的当前位置为中心（drv 层内置），避免起始误差过大
    };
    BSP_ASSERT_APP_CALL(AxisPidLiteInit(&yaw_axis, &yaw_axis_cfg));

    // 注册 BMI088 + 卡尔曼（只注册子模块，Config 时配置硬件）
    BSP_ASSERT_APP_CALL(BMI088KalmanRegister(&bmi088));
    /* 配置 BMI088 + 卡尔曼（标定参数见上面的 s_bmi088_calib）
     * 噪声参数取值的依据见 drvlib_bmi088_kalman.h 头注释与下面的注释 */
    BMI088Kalman_Config_s bmi088_cfg = {
        .imu =
            {
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
                .spi_mode = BSP_DMA_MODE, // 传输方式：DRDY EXTI 里发起，只能用 IT/DMA（INT + BLOCK 被 Config 拒绝）
                .spi_timeout_ms = 10,     // SPI 传输超时(ms)：BLOCK 透传 HAL，IT/DMA 用于等总线就绪
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
         * ⚠ 与 lib_axis_pid_lite 的 vofa_enable(CH vofa_base_ch ~ +11)、app_shoot 的调试块
         *   (CH1~CH4) 共用通道号；要么错开起始通道，要么同一时刻只开一路。
         * 当前为 0：正在整定 app_shoot 的摩擦轮速度环（它占 CH1~CH4 并自己发帧），
         * 两路同时开会每拍发两帧、数据互相覆盖。调完摩擦轮把这里改回 1。 */
        .vofa_enable = 0,

    };
    BSP_ASSERT_APP_CALL(BMI088KalmanConfig(&bmi088, &bmi088_cfg));

    // 云台状态机：上电先「失能」，等 cmd 的第一帧模式再换档（mode_last 零值即 stop，正好对上）
    LibFsmTableConfig_s gimbal_fsm_cfg = {
        .initial_state = gimbal_state_disable_e,
        .transitions = gimbal_fsm_table,
        .transition_count = sizeof(gimbal_fsm_table) / sizeof(gimbal_fsm_table[0]),
        /* 暂无进入/退出动作：本状态机的产出是"哪几轴参与闭环"（见 AppGimbalRun）。
         * 将来要"进档瞬间做一件事"（复位积分、把设定值对齐当前反馈…）就挂在这里。 */
        .state_actions = NULL,
        .state_count = 0,
        .ctx = &gimbal_fsm_ctx,
        .trace_fn = NULL,
    };
    LibFsmTableInit(&gimbal_fsm, &gimbal_fsm_cfg);
    LibFsmTableStart(&gimbal_fsm);
}

ITCM_RAM void AppGimbalRun(float dt, uint64_t time_stamp)
{
    const float dt_s = dt * 0.001f; // APP_TASK_DEF 的 dt 是 ms，drvlib 要秒

    /* 周期重发使能：只发一次时，电机晚于 STM 上电会永远收不到 0xFC（RS05 上电后
     * 还有一段 CALI 不接受 MIT 命令）。无条件按固定周期重发，不依赖 daemon 的在线状态；
     * 与离线回调的重发并存，最多多发几帧，对总线无压力。 */
    if ((time_stamp - gimbal_enable_last_ms) >= (uint64_t)GIMBAL_ENABLE_RESEND_MS)
    {
        gimbal_enable_last_ms = time_stamp;
        DrvDMMotorSendCmd(&pitchdown_motor, DRV_DM_CMD_MOTOR_MODE);
        DrvDMMotorSendCmd(&pitchup_motor, DRV_DM_CMD_MOTOR_MODE);
        DrvRSMotorSendCmd(&yaw_motor, DRV_RS_CMD_MOTOR_MODE);
    }

    // 接收消息
    xQueueReceive(cmd2gimbal_queue_handle, &gimbal_cmd2gimbal_data, 0);

    /* ---- 电机反馈：协议原始量 → drvlib 后处理（多圈/偏置/方向/低通） ----
     * 先喂一次（开环下返回值无用，ref 给 0）把本拍 data 刷新出来供控制律使用；
     * 算完给定后再喂一次拿真正下发量。同一帧按 timestamp_us 去重，不会重复滤波/累圈。
     * 反馈累加与使能无关，失能期间也每拍喂，手动转动后位置保持连续。 */
    const DrvDMMotorData_s pitchdown_data = DrvDMMotorGetData(&pitchdown_motor);
    const DrvDMMotorData_s pitchup_data = DrvDMMotorGetData(&pitchup_motor);
    const DrvRSMotorData_s yaw_data = DrvRSMotorGetData(&yaw_motor);

    const LibMotorFeedback_s pitchdown_fb = {
        .position = pitchdown_data.position,
        .speed = pitchdown_data.speed,
        .current = pitchdown_data.torque, // 最内环语义是电流；DM/RS 这里喂扭矩
        .timestamp_us = pitchdown_data.timestamp_us,
    };
    const LibMotorFeedback_s pitchup_fb = {
        .position = pitchup_data.position,
        .speed = pitchup_data.speed,
        .current = pitchup_data.torque,
        .timestamp_us = pitchup_data.timestamp_us,
    };
    const LibMotorFeedback_s yaw_fb = {
        .position = yaw_data.position,
        .speed = yaw_data.speed,
        .current = yaw_data.torque,
        .timestamp_us = yaw_data.timestamp_us,
    };

    (void)LibMotorSetRef(&pitchdown_ctrl, 0.0f, &pitchdown_fb, dt_s);
    (void)LibMotorSetRef(&pitchup_ctrl, 0.0f, &pitchup_fb, dt_s);
    (void)LibMotorSetRef(&yaw_ctrl, 0.0f, &yaw_fb, dt_s);

    // 计算当前状态
    const float pitchdown_pos = (float)pitchdown_ctrl.data.position;
    /* 上 pitch 相对下 pitch 的关节角（旧代码同一算法，只是数据源换成处理后的量） */
    const float pitchup_pos =
        (float)((pitchup_ctrl.data.position - (double)pitchup_position_0) - (pitchdown_ctrl.data.position - (double)pitchdown_position_min));

    // 换算为 axis lite 的反馈输入（lite 层不依赖 motor，app 侧把处理后的量摊平）
    AxisLiteState_s pitchdown_state = {
        .position = pitchdown_pos,
        .speed = pitchdown_ctrl.data.speed,
        .torque = pitchdown_ctrl.data.current,
    };
    AxisLiteState_s pitchup_state = {
        .position = pitchup_pos,
        .speed = pitchup_ctrl.data.speed,
        .torque = pitchup_ctrl.data.current,
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
        .torque = yaw_ctrl.data.current, // 编码器侧反馈只用于 ID/监控，控制量取自 IMU
    };

    /* yaw 相对底盘前进方向的夹角 θ（编码器关节角，不是世界系航向）：
     * 主动倒下的前置条件 —— 云台要先和底盘对正，折叠时才不会别住车体。
     * 速度也取编码器侧：判断的是“相对底盘不再转动”，与 IMU 世界系 yaw_rate 无关。 */
    const float yaw_rel = Lib_Math_WrapAngleNegPIToPI((float)yaw_ctrl.data.position - chassis_gimbal_offset);
    const uint8_t yaw_aligned = (Lib_Math_Fabs(yaw_rel) < GIMBAL_YAW_ALIGN_EPS) && (Lib_Math_Fabs(yaw_ctrl.data.speed) < GIMBAL_YAW_ALIGN_SPEED);

    /* VOFA 调试输出已移到 drvlib 里：app_gimbal 的 bmi088_cfg 里把
     *   .vofa_enable = 1
     * 打开，drvlib 每帧自己把 IMU 数据写进通道并 VofaSend（通道表在
     * drvlib_bmi088_kalman.h 头注释"VOFA 调试输出"一节，CH1~CH15）。
     * 验证标定：CH13-15 上电后应恒等于 s_bmi088_calib 里的数值（tempco 未标定时）；
     *   看 CH12（温度）与 CH4-6 均值漂移可判断是否还需要温漂标定；
     *   yaw 漂移速率看 CH3 长时间趋势，转动 yaw 时 CH11 应约等于 d(CH3)/dt，
     *   且 CH1-2 基本不动。
     * 注意：帧由 drvlib 发出，app 不要再自己调 VofaSend（会多发一帧）；
     *       而通道号是全局的 —— lib_axis_pid_lite 的 vofa_enable 占用
     *       CH vofa_base_ch ~ +11、app_shoot.c 的调试块占用 CH1~CH4，
     *       要同时看就得把它们错开或用别的通道号；
     *       本 app 眼下是"整定发射就把这里关掉、发射调完再打开"（见下面 vofa_enable） */

    // setref
    /* 状态机：驱动量 = cmd 下发的模式（目标姿态）+ 下 pitch 位置（实际姿态），解出
     * 失能/立起中/直立/倒下中/倒下 —— 见 GimbalFsmUpdate 上方的注释。 */
    GimbalFsmUpdate(gimbal_cmd2gimbal_data.mode, pitchdown_pos);
    gimbal_state_e gimbal_state = (gimbal_state_e)LibFsmTableCurrent(&gimbal_fsm);

    /* 倒下许可：hole 档且 yaw 与底盘对正后锁存；离开 hole 档重新解禁。
     * 一旦锁存，中途 θ 抖动/底盘被挡住也继续把倒下走完，避免停在半路。 */
    if (robot_mode_hole != gimbal_cmd2gimbal_data.mode)
    {
        gimbal_fall_permitted = 0;
    }
    else if (yaw_aligned)
    {
        gimbal_fall_permitted = 1;
    }

    /* 失能之外三轴都参与闭环：pitch 在 falling/lying 也由 axis 模块主动保持/过渡 */
    uint8_t pitch_on = (gimbal_state != gimbal_state_disable_e);
    uint8_t yaw_on = (gimbal_state != gimbal_state_disable_e);

    /* 状态切换时把姿态设定值锚在当前实测位置，避免 target 从旧状态残留值跳变 */
    if (gimbal_state != gimbal_state_last)
    {
        pitchdown_posture_ref = pitchdown_pos;
        pitchup_posture_ref = pitchup_pos;
        gimbal_state_last = gimbal_state;
    }

    // 清零
    pitchup_motor_setref = 0;
    pitchdown_motor_setref = 0;
    yaw_motor_setref = 0;

    if (pitch_on)
    {
        /* 姿态目标：立起/直立时下 pitch 去 max、上 pitch 跟 cmd；
         * 倒下时下 pitch 去 min、上 pitch 相对角回 0（折叠）。
         * falling 下若 yaw 还没对正，两个 ref 停在进入 falling 时的实测位置（主动保持）。 */
        const float slew_max = GIMBAL_POSTURE_SLEW_RATE * dt_s;
        float pitchdown_step = 0.0f;
        float pitchup_step = 0.0f;
        uint8_t pitchup_use_cmd = 0;

        switch (gimbal_state)
        {
        case gimbal_state_rising_e:
        case gimbal_state_stand_e:
            pitchdown_step = GimbalSlewStep(&pitchdown_posture_ref, pitchdown_position_max, slew_max);
            pitchup_use_cmd = 1;
            break;

        case gimbal_state_falling_e:
            if (gimbal_fall_permitted)
            {
                pitchdown_step = GimbalSlewStep(&pitchdown_posture_ref, pitchdown_position_min, slew_max);
                pitchup_step = GimbalSlewStep(&pitchup_posture_ref, 0.0f, slew_max);
            }
            break; // 未对正：ref 保持进入 falling 时的位置

        case gimbal_state_lying_e:
            pitchdown_step = GimbalSlewStep(&pitchdown_posture_ref, pitchdown_position_min, slew_max);
            pitchup_step = GimbalSlewStep(&pitchup_posture_ref, 0.0f, slew_max);
            break;

        case gimbal_state_disable_e:
        default:
            break;
        }

        // setref-pitchdown：axis 模块只取重力前馈（FIXED_TORQUE 阶段 ref 被忽略）
        AxisLiteRef_s pitchdown_ref = {
            .position = pitchdown_posture_ref,
            .speed = (dt_s > 0.0f) ? (pitchdown_step / dt_s) : 0.0f,
            .acceleration = 0.0f,
        };
        pitchdown_motor_setref = AxisPidLiteCalculate(&pitchdown_axis, &pitchdown_state, &pitchdown_ref);

        if (gimbal_state == gimbal_state_lying_e)
        {
            /* 已倒下：关闭重力前馈，下 pitch 松劲靠自重/机械限位停住 */
            pitchdown_motor_setref = 0.0f;
        }
        else
        {
            /* 速度闭环：只在上升/下降阶段参与。
             * 速度给定为常量：上升 +SLEW_RATE、下降 -SLEW_RATE；falling 未获 yaw 对正
             * 许可时给定 0，速度环只剩阻尼。到位由状态机按实际位置判据切换，切走后
             * 速度环退出，不依赖时间去“猜”是否到点。 */
            if ((gimbal_state == gimbal_state_rising_e) || (gimbal_state == gimbal_state_falling_e))
            {
                float pitchdown_v_ref = 0.0f;
                if (gimbal_state == gimbal_state_rising_e)
                {
                    pitchdown_v_ref = GIMBAL_POSTURE_SLEW_RATE;
                }
                else if (gimbal_fall_permitted)
                {
                    pitchdown_v_ref = -GIMBAL_POSTURE_SLEW_RATE;
                }
                pitchdown_motor_setref += GIMBAL_PITCHDOWN_VEL_KV * (pitchdown_v_ref - pitchdown_state.speed);
            }

            /* 最高点锁定：加固定偏置，补偿重力前馈残差 */
            if (gimbal_state == gimbal_state_stand_e)
            {
                pitchdown_motor_setref += GIMBAL_PITCHDOWN_TORQUE_BIAS;
            }
        }

        // setref-pitchup
        AxisLiteRef_s pitchup_ref;
        if (pitchup_use_cmd)
        {
            // 立起/直立：外部设定值来自 cmd（含规划器给的 v/a）
            pitchup_ref.position = gimbal_cmd2gimbal_data.pitch_x;
            pitchup_ref.speed = gimbal_cmd2gimbal_data.pitch_v;
            pitchup_ref.acceleration = gimbal_cmd2gimbal_data.pitch_a;
        }
        else
        {
            // 姿态过渡/倒下保持：限速目标 + 对应速度前馈
            pitchup_ref.position = pitchup_posture_ref;
            pitchup_ref.speed = (dt_s > 0.0f) ? (pitchup_step / dt_s) : 0.0f;
            pitchup_ref.acceleration = 0.0f;
        }
        pitchup_motor_setref = AxisPidLiteCalculate(&pitchup_axis, &pitchup_state, &pitchup_ref);

        /* 上 pitch 单向耦合：只在最高点锁定时生效。
         * 上 pitch 输出为负时，其反作用需要下 pitch 同量补偿；
         * 上升/下降/倒下阶段不叠加，避免过渡过程中两轴互相拖拽。 */
        if ((gimbal_state == gimbal_state_stand_e) && (pitchup_motor_setref < 0.0f))
        {
            pitchdown_motor_setref += -pitchup_motor_setref * PITCH_UP2DOWN_KP;
        }
    }
    // setref-yaw
    if (yaw_on)
    {
        // 外部设定值来自 cmd（NORMAL 阶段使用；当前 TUNE 阶段内部正弦，此参数被忽略）
        AxisLiteRef_s yaw_ref = {
            .position = gimbal_cmd2gimbal_data.yaw_x,
            .speed = gimbal_cmd2gimbal_data.yaw_v,
            .acceleration = gimbal_cmd2gimbal_data.yaw_a,
        };
        yaw_motor_setref = AxisPidLiteCalculate(&yaw_axis, &yaw_state, &yaw_ref);
    }

    // send：开环下返回值 = ref × motor_direction（方向修正仍在控制核里统一做）
    const float pitchup_out = LibMotorSetRef(&pitchup_ctrl, pitchup_motor_setref, &pitchup_fb, dt_s);
    const float pitchdown_out = LibMotorSetRef(&pitchdown_ctrl, pitchdown_motor_setref, &pitchdown_fb, dt_s);
    const float yaw_out = LibMotorSetRef(&yaw_ctrl, yaw_motor_setref, &yaw_fb, dt_s);

    DrvDMMotorSetRef(&pitchup_motor, pitchup_out);
    DrvDMMotorSetRef(&pitchdown_motor, pitchdown_out);
    DrvRSMotorSetRef(&yaw_motor, yaw_out);
    DrvDMMotorSend(&pitchdown_motor);
    DrvDMMotorSend(&pitchup_motor);
    DrvRSMotorSend(&yaw_motor);

    /* 回传云台反馈给 cmd（规划器需要当前位置/速度；pitch_down 供视觉回传下pitch位姿角）。
     * yaw 与 yaw_vel 回传的是 IMU 世界系量：必须与上面 yaw 轴控制用的反馈同源，
     * 否则 cmd 的规划器会拿"相对底盘"的锚点去规划"世界系"的目标，每帧都差一个底盘航向。
     * 同时视觉下发的 yaw 本就是世界系（见 app_cmd send_gimbal 的视觉分支），回传也对齐。
     * yaw_motor_* 另外回传编码器关节角（世界系量丢掉了"云台相对底盘"这一信息）：
     *   cmd 用它在本地算实测 θ 并旋转摇杆向量；底盘不接收实测角，而是自己从共享
     *   CAN1 读同一颗 RS05。帧里的 gimbal2chassis_data.yaw 只是目标偏角 θ_ref。
     *   因此这里必须回传编码器相对角，不能拿世界系 yaw_position 代替
     *   （底盘一转它就变，目标角播种/向量旋转都会错）。 */
    gimbal2cmd_data_t gimbal2cmd_data = {
        .state = gimbal_state, // 云台实际姿态（cmd 可拿去判"云台到没到位/在不在动作中"）
        .pitch_position = pitchup_pos,
        .pitch_vel = pitchup_ctrl.data.speed,
        .yaw_position = imu_data.euler.yaw,
        .yaw_vel = imu_data.yaw_rate,
        .pitch_down_position = pitchdown_pos,
        .yaw_motor_position = (float)yaw_ctrl.data.position,
        .yaw_motor_vel = yaw_ctrl.data.speed,
    };
    xQueueOverwrite(gimbal2cmd_queue_handle, &gimbal2cmd_data);

    /* VOFA：axis 模块已在本拍把通道写完（vofa_enable=1 的轴），这里统一发一帧。
     * 必须放在所有 VofaSetChannel 之后、且全工程只能有一个发送者（见文件头的开关说明）。 */
#if GIMBAL_VOFA_SEND
    VofaSend();
#endif
}
