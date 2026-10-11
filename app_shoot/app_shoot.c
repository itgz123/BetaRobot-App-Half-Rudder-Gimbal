#include "app_shoot.h"
#include "app_shoot_def.h" // 发射整定参数（宏 + 驱动律说明），本文件只放控制逻辑
#include "app_cfg.h"
#include "app.h"
#include "app_cmd_shoot.h" // cmd2shoot / shoot2cmd 队列句柄与数据结构
#include "robot_def.h"     // gimbal限位/速度/加速度宏 + TRIGGER_STEP_RAD（每发机构转角）
//
#include "drvs_djimotor_broadcast.h" // DJI 广播纯协议层
#include "drvlib_motor.h"            // 三级级联 PID 控制核
#include "lib_fsm_table.h"           // 表驱动状态机（拨弹状态机）
#include "lib_traj.h"                // 单轴轨迹核心（段/轨迹/按时间采样）
#include "lib_traj_jerk.h"           // jerk 受限点到点规划器
#include "drv_vofa.h"
//
#include "bsp_sys_status.h"
#include "bsp_freertos.h"
//
#include <stdbool.h> // 状态机守卫返回 bool

// 变量
// 实例
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(friction_motor);    // 摩擦轮 M3508 (C620)
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(friction_motor_group); // 摩擦轮广播组（一条总线一组）
DRVLIB_MOTOR_INSTANCE_DEF(friction_motor_ctrl);          // 摩擦轮速度环（三级 PID 核的中间环）
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(trigger_motor);     // 拨弹 M3508 (C620)
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(trigger_motor_group);  // 拨弹广播组（CAN_1，独立于摩擦轮组）
DRVLIB_MOTOR_INSTANCE_DEF(trigger_motor_ctrl);           // 拨弹 速度环（三级 PID 核的中环；无位置环）
// 通信
static cmd2shoot_data_t shoot_cmd2shoot_data; // cmd-shoot
static shoot2cmd_data_t shoot_shoot2cmd_data; // shoot-cmd
// 数据
static float friction_motor_setref = 0; // 摩擦轮目标转速 (rad/s)：本阶段无来源，恒 0
static float trigger_friction_ff = 0;   // 拨盘摩擦前馈 (A)：speed_feedforward 指针指向它，本阶段恒 0

// 拨盘参考：单发/三发/连发都走 lib_traj_jerk 点到点轨迹，**终点必落在发数网格上**
LIB_TRAJ_INSTANCE_DEF(trigger_traj);     // 本段（一步 / 三发一程 / 连发一整段 / 收尾段）的预计算轨迹
static LibTraj_Output_s trigger_ref_out; // 上一拍参考输出（position/velocity/acceleration）
static float trigger_angle_ff = 0;       // 角度环(速度设定值域)前馈 = 参考速度；空闲恒 0
static float trigger_target_pos = 0;     // 本段轨迹终点 (rad, app 正向, 多圈)：恒为 起始 + N×TRIGGER_STEP_RAD
static uint8_t trigger_planned = 0;      // 当前是否有有效轨迹在跑
static uint8_t trigger_active = 0;       // 上一拍是否在主动驱动机构（退出沿清一次 PID 并锁位）
// 连发收尾
static float trigger_shot_origin = 0; // 本段的发数零点（规划时锁在起始实测位置），收尾轨迹据此对齐整数发
static uint8_t trigger_burst = 0;     // 当前跑的是连发长轨迹（松手后需再走一条收尾轨迹才真正停）
static uint8_t trigger_settling = 0;  // 正在走连发的收尾轨迹（终点为整数发边界、终点静止）

/*============================================
 *        拨弹状态机（lib_fsm_table）
 *   状态：关 / 单发 / 三发 / 连发 / 停止
 *   事件：ch7 上升沿 / ch7 下降沿 / 本步轨迹走完（step_done）
 *   设计要点：
 *   - 发射模式（ch9）只在「关 + ch7 上升沿」这一拍读取，之后锁在所进的状态里；中途改
 *     ch9（甚至改模式）都不影响正在走的这一发。
 *   - **一旦发射就要打完**（2026-10-11 用户定）：单发走完 1 发、三发走完 3 发，中途收到 ch7
 *     下降沿都**不打断** —— 打完那一刻在"走完一发"规则里看 ch7 电平：已断开回「关」、
 *     仍闭合进「停止」等断开。只有连发是"运动中可停"：收到 ch7 下降沿立即回「关」，
 *     由 AppShootRun 的收尾轨迹停在整数发边界上。
 *   - "走完一发"的判据只有**一个**：整段轨迹 finished —— 三种模式都是点到点轨迹、终点都落在
 *     整数发网格上（单发 1 发、三发 3 发、连发 500 发），finished 就是"这一发/这三发打完"。
 *     连发跑完 500 发在「连发」里原地续段，不产生 step_done。
 *   - 进入发弹状态时置 ctx->plan_request，AppShootRun 据此规划本段的**整条**轨迹。
 *   - ctx->continuous 由进入动作置位：三发/连发 = 1（一条长轨迹），单发/关/停止 = 0（单步）。
 *============================================*/
typedef enum : uint8_t
{
    trigger_state_off_e = 0, // 关：ch7 断开，不动作
    trigger_state_single_e,  // 单发：走完 1 发（点到点，起停包络）
    trigger_state_triple_e,  // 三发：连续拨，走满 3 份行程后停
    trigger_state_auto_e,    // 连发：连续拨，ch7 断开才停
    trigger_state_stop_e,    // 停止：已打完/已停稳，等 ch7 断开
    trigger_state_count_e,   // 状态值域上界（= 进入动作表项数）
} trigger_state_e;

typedef enum : uint8_t
{
    trigger_ev_fire_on_e = 0, // ch7 0→1
    trigger_ev_fire_off_e,    // ch7 1→0
    trigger_ev_step_done_e,   // 走完一发：单发=轨迹走完；三发/连发=累计转角满一份 TRIGGER_STEP_RAD
} trigger_event_e;

/* 状态机上下文：发射模式在每拍派发前由 TriggerFsmUpdate 刷新 */
typedef struct
{
    fire_mode_e fire_mode; // 本拍 cmd 下发的发射模式（off 同时表示"ch7 断开"）
    uint8_t fire_last;     // 上一拍 ch7 电平（上升/下降沿检测）
    uint8_t plan_request;  // 1 = 需要为新的一步重新规划轨迹（进入发弹状态时置位；由 AppShootRun 消费）
    uint8_t continuous;    // 1 = 本状态走"连续转动"（三发/连发，整段一条轨迹），0 = 单发一步
} TriggerFsmCtx_s;

static TriggerFsmCtx_s trigger_fsm_ctx;
static LibFsmTableInstance_s trigger_fsm;

/*---------------- 状态动作（进入/退出） ----------------*/
/* 进入任何状态都请求规划新的一步：让新状态的 E_STEP_DONE 从机构当前位置重新起算
 * （AppShootRun 消费 plan_request 后会清零；停/关状态也置位，但那时不在发弹、不会被消费） */
static void TriggerFsmEnterState(void *ctx)
{
    TriggerFsmCtx_s *c = ctx;
    c->plan_request = 1;
    c->continuous = 0; // 关/停止：点到点路径（其实那时也不驱动，置 0 只为状态干净）
}
static void TriggerFsmEnterSingle(void *ctx)
{
    TriggerFsmEnterState(ctx); // 单发：一步一条轨迹，continuous 保持 0
}
static void TriggerFsmEnterTriple(void *ctx)
{
    TriggerFsmEnterState(ctx);
    ((TriggerFsmCtx_s *)ctx)->continuous = 1; // 三发：一条 3 发长的整段轨迹，中间不停
}
static void TriggerFsmEnterAuto(void *ctx)
{
    TriggerFsmEnterState(ctx);
    ((TriggerFsmCtx_s *)ctx)->continuous = 1; // 连发：一条超长轨迹，ch7 断开后另走收尾轨迹停
}

/*---------------- 守卫 ----------------*/
/* 守卫必须无副作用且幂等：同 from/event 的多条规则会依次求值（见 lib_fsm_table.h） */
static bool TriggerFsmGuardSingle(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((TriggerFsmCtx_s *)ctx)->fire_mode == fire_mode_single_e;
}
static bool TriggerFsmGuardTriple(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((TriggerFsmCtx_s *)ctx)->fire_mode == fire_mode_triple_e;
}
static bool TriggerFsmGuardAuto(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((TriggerFsmCtx_s *)ctx)->fire_mode == fire_mode_auto_e;
}
static bool TriggerFsmGuardFireOff(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((TriggerFsmCtx_s *)ctx)->fire_mode == fire_mode_off_e;
}
/*---------------- 转移表（表顺序 = 优先级） ----------------*/
static const LibFsmTableTransition_s trigger_fsm_table[] = {
    /* 1~3. 关 + ch7 上升沿 → 按 ch9 解出的模式进对应状态（三条同 from/event，守卫依次求值） */
    {trigger_state_off_e, trigger_ev_fire_on_e, trigger_state_single_e, TriggerFsmGuardSingle, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_off_e, trigger_ev_fire_on_e, trigger_state_triple_e, TriggerFsmGuardTriple, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_off_e, trigger_ev_fire_on_e, trigger_state_auto_e, TriggerFsmGuardAuto, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    /* 4. 本步走完 —— 就是"整段轨迹跑完"这**一个**事件（轨迹终点必落在整数发网格上）：
     *    单发 = 1 发一条轨迹、三发 = 3 发一条轨迹，finished 即"打完"；连发是 500 发的长轨迹，
     *    跑完在「连发」里原地续段（驱动层不喂 step_done），故这里没有连发规则。
     *    同事件先看 ch7 电平：已断开回「关」、仍闭合进「停止」等断开 —— 单发/三发完全同构。
     *    ⚠ 不要退回"逐发递减计数"（拿参考位置和发数阈值比）：行程恰为整数发时，浮点边界会让
     *    最后一发数不上，状态机就永久锁死在发弹态（2026-10-11 实测三发打完卡死即此因，
     *    参考终点 40.2124 rad vs 3×13.4041284 = 40.212385 rad，只差 1e-5）。 */
    {trigger_state_single_e, trigger_ev_step_done_e, trigger_state_off_e, TriggerFsmGuardFireOff, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_single_e, trigger_ev_step_done_e, trigger_state_stop_e, NULL, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_triple_e, trigger_ev_step_done_e, trigger_state_off_e, TriggerFsmGuardFireOff, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_triple_e, trigger_ev_step_done_e, trigger_state_stop_e, NULL, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    /* 5. ch7 下降沿 → 关。**单发/三发都没有这条**：一旦发射就要打满（1 发 / 3 发），中途松手
     *    不打断，打完由上面第 4 组规则看电平决定回关还是等断开。只有连发运动中可停 —— 松手
     *    立即回「关」，驱动层再走一条"到整数发边界"的收尾轨迹停下（见 AppShootRun）。 */
    {trigger_state_stop_e, trigger_ev_fire_off_e, trigger_state_off_e, NULL, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_auto_e, trigger_ev_fire_off_e, trigger_state_off_e, NULL, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
};
LIB_FSM_TABLE_CHECK_TABLE(trigger_fsm_table);

static const LibFsmTableStateActions_s trigger_fsm_actions[] = {
    [trigger_state_off_e] = {TriggerFsmEnterState, NULL},     // 关
    [trigger_state_single_e] = {TriggerFsmEnterSingle, NULL}, // 单发
    [trigger_state_triple_e] = {TriggerFsmEnterTriple, NULL}, // 三发
    [trigger_state_auto_e] = {TriggerFsmEnterAuto, NULL},     // 连发
    [trigger_state_stop_e] = {TriggerFsmEnterState, NULL},    // 停止
};
LIB_FSM_TABLE_CHECK_ACTIONS(trigger_fsm_actions, (uint32_t)trigger_state_count_e);

/*---------------- 每拍推进 ----------------*/
/* 事件来源：ch7 电平（cmd 把 ch7 与 ch9 一起编码进 fire_mode）→ 上升/下降沿；
 *           "本步轨迹走完"由 AppShootRun 喂进来（TriggerFsmNotifyStepDone）。
 * 两件事分开做：先 Update 刷 ch7 边沿（可能进入发弹状态、置 plan_request），
 * AppShootRun 再走一拍轨迹，最后按"轨迹是否 finished"决定要不要 NotifyStepDone。 */
static void TriggerFsmUpdate(void)
{
    TriggerFsmCtx_s *c = &trigger_fsm_ctx;

    /* 刷新本拍输入：发射模式（off 即 ch7 断开，二者由 cmd 编在同一字段里，无需另传电平） */
    c->fire_mode = shoot_cmd2shoot_data.fire_mode;

    /* ch7 边沿 */
    uint8_t fire_now = (c->fire_mode != fire_mode_off_e) ? 1 : 0;
    if (fire_now != c->fire_last)
    {
        LibFsmTableDispatch(&trigger_fsm, (fire_now != 0) ? trigger_ev_fire_on_e : trigger_ev_fire_off_e, NULL);
        c->fire_last = fire_now;
    }
}

/* 本步轨迹走完：喂一次 step_done（单发 → 停止/关；三发/连发 → 起下一发）。
 * 一拍至多一次；停在关/停止时查表无人接手，返回值忽略即可。 */
static void TriggerFsmNotifyStepDone(void)
{
    LibFsmTableDispatch(&trigger_fsm, trigger_ev_step_done_e, NULL);
}

/* 当前是否"发弹中"（拨盘该跑规划轨迹）：单发/三发/连发都跑；关/停止不跑。
 * 停止态是"这一发已走完、等 ch7 断开"，必须停。 */
static uint8_t TriggerFsmIsFiring(void)
{
    LibFsmTableState_t s = LibFsmTableCurrent(&trigger_fsm);
    return (uint8_t)((s == trigger_state_single_e) || (s == trigger_state_triple_e) || (s == trigger_state_auto_e));
}

void AppShootInit(void)
{
    // 注册 CAN 实例并挂入广播组
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&friction_motor));
    DrvsDJIMotorBroadcastConfig_s friction_motor_cfg = {
        .group = &friction_motor_group,
        .can_e = CAN_2,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 7,
        .torque_constant = 1, // 转矩常数 (Nm/A)
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&friction_motor, &friction_motor_cfg));

    // 控制核：速度环（ref = rad/s → PID → 电流）
    DrvlibMotor_Config_s friction_motor_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_SPEED,         // 速度闭环
        .position_mode = DRVLIB_MOTOR_POS_CONTINUOUS, // 不用位置环（连续模式，不引入限幅）
        .angle_limit_max = 0,
        .angle_limit_min = 0,
        .position_span = 6.2831853f, // 单圈 2π（M3508 编码器跨度）
        .position_offset = 0,
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE, // 速度/电流低通
        .lpf_rc = 0.02f,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_DISABLE, // 前馈来源
        .speed_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .angle_feedforward_ptr = NULL, // 前馈指针
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .pid =
            {
                [DRVLIB_MOTOR_STAGE_SPEED] =
                    {
                        /* kp=0.2：去饱和后的闭环时间常数 τ_cl 基本由 kp 决定。实测 kp=0.1 时
                         * τ_cl≈78 ms（≈12.8 rad/s 带宽），而 20 ms 速度 LPF 的极点 ≈50 rad/s 还在
                         * 2 个数量级之外 —— 带宽瓶颈是 kp、不是滤波，故 kp 翻倍把 τ_cl 压到 ~40 ms，
                         * 抗扰动带宽约翻倍（这是"抗扰动"唯一的直接杠杆）。
                         * 上升段（t10-90≈280 ms）本来就顶在驱动限幅上、与 kp 无关；实测 ripple 仅
                         * 0.16 rad/s，反馈干净，翻倍后信噪比仍充足。 */
                        .kp = 0.2, // 比例系数
                        /* ki：维持积分零点在闭环带宽的 1/5 —— T_i = 5·τ_cl ≈ 200 ms，
                         * ki = kp/T_i = 0.2/0.2 = 1.0 (1/s)。kp 翻倍而 τ_cl 减半，故 ki 同步翻倍。 */
                        .ki = 1.0, // 积分系数 (1/s)
                        .kd = 0,   // 微分系数：反馈干净（ripple 仅 0.2~0.3 rad/s）、微分只会放大量化噪声
                        /* 积分项硬限幅 (A)：必须 >0 —— 传 0 会让 f_Integral_Limit 把 i_out 永久清零，
                         * 积分被静默禁用、退回纯 P（本项目踩过这个坑） */
                        .integral_limit = FRICTION_I_LIMIT, // 积分限幅阈值 (A)
                        .coef_a = FRICTION_VARINT_A,        // 变速积分衰减宽度 A (rad/s)，0 = 禁用
                        .coef_b = FRICTION_VARINT_B,        // 变速积分全积分带宽 B (rad/s)
                        .d_lpf_rc = 0,                      // 微分滤波时间常数 RC (0 = 禁用)
                        .out_lpf_rc = 0,                    // 输出滤波时间常数 RC (0 = 禁用)
                        .deadband = 0,                      // 死区范围 (0 = 禁用)
                        /* 输出(电流)限幅：既是给驱动限幅、也是抗积分饱和的判据。
                         * 原配置 out_max/min=0 且没开 OUTPUT_LIMIT，输出无上限 ——
                         * 数据里 out_pk 冲到 76.69 A 就是证据：上升段误差大、电流顶到驱动
                         * 20 A 限幅后 PID 自己还在往上积，驱动限幅以外全是饱和区。 */
                        .out_max = FRICTION_I_MAX,  // 输出上限 (A)，需要 PID_ENABLE_OUTPUT_LIMIT
                        .out_min = -FRICTION_I_MAX, // 输出下限 (A)
                        /* 三个限幅/分离掩码合起来是「大误差不积、饱和不积、幅值封顶」：
                         * CHANGING_INTEGRATION 管误差幅值轴（见 FRICTION_VARINT_* 说明），
                         * INTEGRAL_LIMIT + OUTPUT_LIMIT 一起管输出饱和轴（缺一不可：前者是
                         * 调用开关，后者的判据在 f_Integral_Limit 内）。TRAPEZOID_INTEGRAL 由
                         * Config 自动补，这里显式写出保持一致。 */
                        .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL | PID_ENABLE_OUTPUT_LIMIT | PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_CHANGING_INTEGRATION,
                    },
            },
        .dt_max = 0.01f, // 单帧 dt 上限 (s)
    };
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&friction_motor_ctrl, &friction_motor_ctrl_cfg));

    DrvlibMotorEnable(&friction_motor_ctrl);

    // 拨弹电机：注册 CAN 实例并挂入广播组
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&trigger_motor));
    DrvsDJIMotorBroadcastConfig_s trigger_motor_cfg = {
        .group = &trigger_motor_group,
        .can_e = CAN_1,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 1,
        .torque_constant = 1, // 转矩常数 (Nm/A)
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&trigger_motor, &trigger_motor_cfg));

    /* 控制核：**位置环 + 速度环**（ANGLE|SPEED），设定值来自 lib_traj_jerk 的规划轨迹：
     * 外环 ANGLE 把轨迹 position 当设定值、输出速度设定值；中环 SPEED 再输出电流。
     * 轨迹 velocity 经 angle_feedforward_ptr 前馈进外环（量纲 = 外环输出 = 速度设定值 rad/s，正好）。
     * 位置模式用 CONTINUOUS：轨迹是**多圈绝对角**，不能 WRAP、也不设限位。
     * 摩擦/加速度前馈按计划留到下一轮（speed/current 两路仍 DISABLE，speed 指针先挂着备接）。 */
    DrvlibMotor_Config_s trigger_motor_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED, // 位置环 + 速度环
        .position_mode = DRVLIB_MOTOR_POS_CONTINUOUS,                   // 多圈绝对角，不限幅不归一化
        .angle_limit_min = 0,
        .angle_limit_max = 0,
        .position_span = 6.2831853f, // 单圈 2π（M3508 编码器跨度），用于多圈累加
        .position_offset = 0,
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE, // 速度/电流低通
        /* 速度低通 5 ms（由 20 ms 下调）：极限环主频 6.5 Hz（41 rad/s）处，20 ms 一阶
         * 低通要吃掉 atan(41×0.02)=39° 相位，是环路里最大的一处滞后。拨盘编码器反馈本身
         * 很干净（静止段 meas 恒为 0.000，运行纹波仅 0.16 rad/s），没必要滤那么狠。
         * 波特图上看，把 4 倍相位（≈28°）还给环路，是提高相位裕度最直接的一手。 */
        .lpf_rc = 0.005f,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_EXTERNAL, // 外环前馈 = 轨迹速度 (rad/s)
        .speed_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,  // 摩擦前馈下一轮再接
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .angle_feedforward_ptr = &trigger_angle_ff,
        .speed_feedforward_ptr = &trigger_friction_ff, // 预留（DISABLE 不读，保持指针有效即可）
        .current_feedforward_ptr = NULL,
        .pid =
            {
                /* 位置环（最外）：只上 P，输出 = 速度设定值 (rad/s)。限幅 TRIGGER_ANGLE_VMAX
                 * 必须 > 轨迹 vmax，否则速度前馈占满权限、比例项无法纠偏（见宏定义处说明）。 */
                [DRVLIB_MOTOR_STAGE_ANGLE] =
                    {
                        .kp = TRIGGER_ANGLE_KP,                  // 比例系数 (1/s)
                        .ki = 0,                                 // 首轮空载不接积分（按计划下一轮再上）
                        .kd = 0,                                 // 微分系数
                        .integral_limit = TRIGGER_ANGLE_I_LIMIT, // 必须 >0，见宏定义处说明
                        .out_max = TRIGGER_ANGLE_VMAX,           // 输出上限 (rad/s，速度设定值)
                        .out_min = -TRIGGER_ANGLE_VMAX,          // 输出下限 (rad/s)
                        .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL | PID_ENABLE_OUTPUT_LIMIT | PID_ENABLE_INTEGRAL_LIMIT,
                    },
                // 速度环：与摩擦轮同一套算法（变速积分 + 输出/积分限幅），量纲按拨盘
                [DRVLIB_MOTOR_STAGE_SPEED] =
                    {
                        /* kp=0.08。2026-10-10 实测：目标 20 rad/s 稳态是**持续极限环**，
                         * 峰峰包络 35.4→8.4→6.9→6.2→7.8→7.4→7.1→8.3 —— 阶跃暂态 2.6 s 内
                         * 衰减到 ~7，之后 18 s 平在 7~8 不衰减（真极限环，不是欠阻尼暂态）。
                         * 关键：kp 从 0.2 降到 0.08，主频 13.2→6.5 Hz（正比于 kp，说明在环路
                         * 穿越处），但**幅值几乎不变**。幅值不随增益走、只有频率随增益走，是
                         * 非线性环节（摩擦/量化/死区）撑着极限环的签名，不是纯相位裕度问题。
                         * 故 kp 维持 0.08，改从三处下手：降速度 LPF（还相位）、降 ki（减积分
                         * 猎振）、改小前馈（消除被积分器反向抵消的恒定偏置）。 */
                        .kp = 0.08, // 比例系数
                        /* ki 由 1.0 降到 0.3：积分器零点是 ki/kp=12.5 rad/s（2 Hz），落在极限环
                         * 频率（6.5 Hz）之下 —— 积分项在振荡频段仍贡献 atan(41/12.5)=73° 之外的
                         * 那部分滞后，且数据里 i_out 一直在慢慢漂（积分器在猎振）。降到 0.3 后
                         * 零点 3.75 rad/s（0.6 Hz），振荡频段积分项近似"纯 P"，滞后减到 ~5°。
                         * 稳态差本就不大（1.5~2.8%），且 DC 摩擦已由前馈接管，不需要大 ki。 */
                        .ki = 0.3,                       // 积分系数 (1/s)
                        .kd = 0,                         // 微分系数
                        .integral_limit = TRIGGER_I_MAX, // 积分限幅（防饱和）
                        .coef_a = TRIGGER_VARINT_A,      // 变速积分衰减宽度 A (rad/s)
                        .coef_b = TRIGGER_VARINT_B,      // 变速积分全积分带宽 B (rad/s)
                        .out_max = TRIGGER_I_MAX,        // 输出上限 (A)
                        .out_min = -TRIGGER_I_MAX,       // 输出下限 (A)
                        .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL | PID_ENABLE_OUTPUT_LIMIT | PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_CHANGING_INTEGRATION,
                    },
            },
        .dt_max = 0.01f, // 单帧 dt 上限 (s)
    };
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&trigger_motor_ctrl, &trigger_motor_ctrl_cfg));

    DrvlibMotorEnable(&trigger_motor_ctrl);

    // 拨弹状态机：初始「关」（ch7 断开）。Start 会跑一次「关」的进入动作（置 plan_request）
    LibFsmTableConfig_s trigger_fsm_cfg = {
        .initial_state = trigger_state_off_e,
        .transitions = trigger_fsm_table,
        .transition_count = sizeof(trigger_fsm_table) / sizeof(trigger_fsm_table[0]),
        .state_actions = trigger_fsm_actions,
        .state_count = trigger_state_count_e,
        .ctx = &trigger_fsm_ctx,
        .trace_fn = NULL,
    };
    LibFsmTableInit(&trigger_fsm, &trigger_fsm_cfg);
    LibFsmTableStart(&trigger_fsm);
}

ITCM_RAM void AppShootRun(float dt, uint64_t time_stamp)
{
    (void)time_stamp;
    // 1. 接收
    xQueueReceive(cmd2shoot_queue_handle, &shoot_cmd2shoot_data, 0);

    // 2. 控制
    /* ---- 摩擦轮：使能满速 / 失能直接给 0 ----
     * 使能：跑速度环（本阶段目标恒 = FRICTION_SPEED_FULL），反馈喂控制核、输出即电流；
     * 失能：不跑控制核，直接 0 A，并清一次 PID —— 否则冻结的积分会带着旧值，下次使能
     *       起步就是一脚踢。两种状态下电机都在收帧，C620 不掉线。
     * 机械方向取反（2026-10-10 看实机定，原为正向）：摩擦轮"正转"应为另一个物理旋向。
     * 手法与拨盘同一套 —— **反馈(position/speed/current)与输出一起取负**：等于重定义正方向，
     * 闭环仍是负反馈、只是物理旋向翻转。⚠ 只反输出会变成正反馈（别那么改）。 */
    DrvsDJIMotorBroadcastData_s fm = DrvsDJIMotorBroadcastGetData(&friction_motor);
    if (shoot_cmd2shoot_data.enable)
    {
        friction_motor_setref = FRICTION_SPEED_FULL;
        DrvlibMotorFeedback_s friction_fb = {
            .position = -fm.position,
            .speed = -fm.speed,
            .current = -fm.current,
            .timestamp_us = fm.timestamp_us,
        };
        /* app 拿到的 dt 是**毫秒**（见 bsp_app.h 的 APP_TASK_DEF 语义），控制核要秒 */
        float friction_out = DrvlibMotorSetRef(&friction_motor_ctrl, friction_motor_setref, &friction_fb, dt * 0.001f);
        DrvsDJIMotorBroadcastSetRef(&friction_motor, -friction_out);
    }
    else // 之后考虑刹车
    {
        friction_motor_setref = 0.0f;
        DrvlibMotorReset(&friction_motor_ctrl); // 清 PID（含积分）与 LPF 状态，不动 enable 标志
        DrvsDJIMotorBroadcastSetRef(&friction_motor, 0.0f);
    }
    DrvsDJIMotorBroadcastGroupSend(&friction_motor_group);

    // ---- 拨盘：拨弹状态机 + jerk 受限轨迹位置环 ----
    /* 机械方向取反（2026-10-10 看实机定）：拨盘"正转"应为另一个物理旋向，故**反馈与输出一起取负**
     * （重定义正方向，闭环仍是负反馈；只反输出会变成正反馈，别那么改）。轨迹位置/速度都在这个
     * "app 正向"帧里，规划起点直接用控制核处理后的**多圈**位置 data.position（已含取负与多圈累加）。 */
    DrvsDJIMotorBroadcastData_s t = DrvsDJIMotorBroadcastGetData(&trigger_motor);
    DrvlibMotorFeedback_s trigger_fb = {
        .position = -t.position,
        .speed = -t.speed,
        .current = t.current,
        .timestamp_us = t.timestamp_us,
    };

    /* 状态机先刷 ch7 边沿（可能进入发弹状态并置 plan_request），再决定本拍怎么走 */
    TriggerFsmUpdate();
    const float trigger_dt_s = dt * 0.001f; // app 拿到的 dt 是毫秒，控制核要秒
    uint8_t firing = TriggerFsmIsFiring();
    uint8_t fb_valid = (t.timestamp_us != 0) ? 1 : 0; // 有 CAN 反馈才敢用位置/速度
    uint8_t burst_now = (uint8_t)(LibFsmTableCurrent(&trigger_fsm) == trigger_state_auto_e);
    uint8_t drive = 0; // 本拍是否在主动驱动机构（决定要不要清 PID、要不要锁位）

    if (firing && fb_valid)
    {
        /* ---------- 发弹中：跑本段整条轨迹（单发/三发/连发都是点到点，终点必在发数网格上） ---------- */
        drive = 1;
        if (!trigger_planned || trigger_fsm_ctx.plan_request)
        {
            /* 规划本段：从**当前实测**位置/速度出发，终点 = 起点 + N 份 TRIGGER_STEP_RAD。
             *   N：单发 1、三发 3、连发 TRIGGER_BURST_SHOTS（长轨迹，松手后另走一条收尾轨迹）。
             *   vmax：连发取射频换算值（射频就是这么被定下来的）；单发/三发取老代码位置环上限
             *   （122.17 rad/s），它们只有 13.4 / 40.2 rad 的行程，实际由 amax 决定快慢。
             * 起点速度取**实测**：连发收尾后紧接着再来一发时机构还在动，不能让轨迹从 v=0 起步。 */
            float p0 = (float)trigger_motor_ctrl.data.position;
            float v0 = trigger_motor_ctrl.data.speed;
            float span = TRIGGER_STEP_RAD;
            float vmax = TRIGGER_TRAJ_VMAX;
            if (trigger_fsm_ctx.continuous)
            {
                if (burst_now)
                {
                    span = TRIGGER_BURST_SHOTS * TRIGGER_STEP_RAD;
                    vmax = TRIGGER_SPIN_SPEED;
                }
                else
                {
                    span = 3.0f * TRIGGER_STEP_RAD;
                }
            }
            trigger_burst = burst_now;
            trigger_settling = 0;
            trigger_shot_origin = p0;
            trigger_target_pos = p0 + span;

            LibTrajJerkPlan_Request_s req = {
                .p0 = p0,
                .v0 = v0,
                .a0 = 0.0f,
                .p1 = trigger_target_pos,
                .v1 = 0.0f,
                .a1 = 0.0f,
                .vmax = vmax,
                .amax = TRIGGER_TRAJ_AMAX,
                .jmax = TRIGGER_TRAJ_JMAX,
            };
            if (LibTrajJerkPlan(&trigger_traj, &req) == LIB_TRAJ_JERK_PLAN_OK)
            {
                trigger_planned = 1;
                LibTrajStep(&trigger_traj, 0.0f, &trigger_ref_out); // 本拍输出轨迹起点
            }
            else
            {
                /* 理论上不会失败（起止状态与极限都合法）：本拍不推进、锁当前位置，下拍重试 */
                trigger_planned = 0;
                trigger_ref_out.position = p0;
                trigger_ref_out.velocity = 0.0f;
                trigger_ref_out.acceleration = 0.0f;
            }
            trigger_fsm_ctx.plan_request = 0;
        }
        else
        {
            LibTrajStep(&trigger_traj, trigger_dt_s, &trigger_ref_out);
        }
        trigger_angle_ff = trigger_ref_out.velocity; // 外环前馈 = 轨迹速度 (rad/s)

        if (trigger_ref_out.finished)
        {
            /* 整段轨迹跑完。**这就是唯一的收尾判据** —— 规划时 span = N×TRIGGER_STEP_RAD，
             * 终点必落在整数发网格上，所以 finished 判据最干净、没有浮点边界问题（曾用"参考
             * 行程累计满一份 TRIGGER_STEP_RAD"数发，行程恰为整数发时最后一发数不上，三发打完
             * 卡死在发弹态，见转移表第 4 组注释）。
             *   单发/三发：一条轨迹 = 打完这 1 发/3 发 → 喂 step_done 让状态机收尾；
             *   连发：500 发的长轨迹跑完还没松手 → 清 planned，下一拍从当前位置续下一条。 */
            if (trigger_burst)
            {
                trigger_planned = 0;
            }
            else
            {
                TriggerFsmNotifyStepDone();
            }
        }
    }
    else if (trigger_burst && fb_valid)
    {
        /* ---------- 连发松手/断开：走一条收尾轨迹停在**整数发边界**，而不是就地硬停 ----------
         * 这一条正是"让整段轨迹的位置是整数发"的落地：收尾目标 = 起始 + k×TRIGGER_STEP_RAD，
         * 终点静止。k 取"已走发数向上取整 + TRIGGER_BURST_STOP_MARGIN"，向上取整保证把当前这发
         * 打完（停在半发位置会让下一发的相位全错），余量保证终点恒远在**前方** —— 否则会规划出
         * "冲过头再倒回来"，而拨盘倒转会退弹。 */
        drive = 1;
        if (!trigger_settling)
        {
            float x = (trigger_ref_out.position - trigger_shot_origin) / TRIGGER_STEP_RAD + TRIGGER_BURST_STOP_MARGIN;
            float k = (float)(int32_t)x; // 向下取整
            if (k < x)
            {
                k += 1.0f; // → 向上取整
            }
            if (k < 1.0f)
            {
                k = 1.0f; // 刚进连发就松手：也把这发打完，不停在半个孔位上
            }
            float p0 = (float)trigger_motor_ctrl.data.position;
            float v0 = trigger_motor_ctrl.data.speed;
            trigger_target_pos = trigger_shot_origin + k * TRIGGER_STEP_RAD;

            LibTrajJerkPlan_Request_s req = {
                .p0 = p0,
                .v0 = v0,
                .a0 = 0.0f,
                .p1 = trigger_target_pos,
                .v1 = 0.0f,
                .a1 = 0.0f,
                .vmax = TRIGGER_TRAJ_VMAX,
                .amax = TRIGGER_TRAJ_AMAX,
                .jmax = TRIGGER_TRAJ_JMAX,
            };
            if (LibTrajJerkPlan(&trigger_traj, &req) == LIB_TRAJ_JERK_PLAN_OK)
            {
                trigger_settling = 1;
                LibTrajStep(&trigger_traj, 0.0f, &trigger_ref_out);
            }
            /* 规划失败：本拍仍按旧参考走，下拍重试 */
        }
        else
        {
            LibTrajStep(&trigger_traj, trigger_dt_s, &trigger_ref_out);
            if (trigger_ref_out.finished)
            {
                /* 停稳。本拍仍用终点参考驱动（drive 保持 1），下一拍才落回"锁实测位置"，
                 * 免得在最后一拍把设定值从终点位置跳回实测位置。 */
                trigger_settling = 0;
                trigger_burst = 0;
                trigger_planned = 0;
            }
        }
        trigger_angle_ff = trigger_ref_out.velocity;
    }
    else
    {
        /* ---------- 空闲（关/停止）或还没有 CAN 反馈 ----------
         * 不驱动；设定值锁在实测位置：位置误差恒 0，只剩速度环对"被手推动"的阻尼，
         * 既不主动找位置也不会失控。 */
        trigger_planned = 0;
        trigger_settling = 0;
        trigger_burst = 0;
        trigger_fsm_ctx.plan_request = 0;
        trigger_angle_ff = 0.0f;
    }

    if (!drive && trigger_active) // 驱动 → 空闲的退出沿：清一次 PID，避免上一轮的积分残留带过来
    {
        DrvlibMotorReset(&trigger_motor_ctrl);
    }
    trigger_active = drive;

    /* 驱动时用上面的参考位置，空闲时设定值锁在实测位置（零误差） */
    float trigger_ref = drive ? trigger_ref_out.position : (float)trigger_motor_ctrl.data.position;
    float trigger_out = DrvlibMotorSetRef(&trigger_motor_ctrl, trigger_ref, &trigger_fb, trigger_dt_s);
    float trigger_cmd = -trigger_out; // app 正方向约定：实发值取负

    DrvsDJIMotorBroadcastSetRef(&trigger_motor, trigger_cmd);
    DrvsDJIMotorBroadcastGroupSend(&trigger_motor_group);

    /* 调试：拨盘**参考位置环**观测通道（本块自己发帧；app_gimbal 的 IMU 那路 VOFA 已关）。
     *   CH1  参考位置 (rad，app 正向，多圈)          —— 位置环给定，三种模式都是同一条 lib_traj 轨迹
     *   CH2  实测位置 (rad，app 正向，多圈)          —— data.position，与 CH1 之差即跟踪误差
     *   CH3  位置误差 = CH1 - CH2 (rad)
     *   CH4  实下发力矩指令 (Nm) = -SetRef 返回值
     *   CH5  参考速度 (rad/s) —— 同时就是**外环速度前馈**。单发一步 13.4 rad、加/减速各需
     *        3.11 rad < 13.4，故够得到 vmax，是"加速-巡航-减速"的梯形包络；连发长轨迹除首尾
     *        两段外恒为巡航值 TRIGGER_SPIN_SPEED（= 射频换算值）
     *   CH6  实测转速 (rad/s，app 正向，低通)        —— data.speed
     *   CH7  参考加速度 (rad/s²)：轨迹加速度（7 段 S 曲线的加/减速相位；巡航段为 0）
     *   CH8  实测电流 (A，app 正向)：本电机 torque_constant=1，数值上 ≈ 力矩
     *   CH9  本拍 dt (ms)：确认控制周期；有跳变说明任务被拖或 CAN 掉帧
     *   CH10 状态机状态 (0关/1单发/2三发/3连发/4停止)
     *   CH11 驱动条件 enable&&fire (0/1)
     *   CH12 原始反馈转速 (rad/s，**电机极性**，未取反)：用于核对方向约定 */
    VofaSetChannel(1, trigger_ref);
    VofaSetChannel(2, (float)trigger_motor_ctrl.data.position);
    VofaSetChannel(3, trigger_ref - (float)trigger_motor_ctrl.data.position);
    VofaSetChannel(4, trigger_cmd);
    VofaSetChannel(5, trigger_angle_ff);
    VofaSetChannel(6, trigger_motor_ctrl.data.speed);
    VofaSetChannel(7, drive ? trigger_ref_out.acceleration : 0.0f);
    VofaSetChannel(8, -t.current); // 实测电流 (A)，取反到 app 正向
    VofaSetChannel(9, dt);         // 本拍 dt (ms)：app 拿到的 dt 就是毫秒
    VofaSetChannel(10, (float)LibFsmTableCurrent(&trigger_fsm));
    VofaSetChannel(11, (float)(shoot_cmd2shoot_data.enable && shoot_cmd2shoot_data.fire));
    VofaSetChannel(12, t.speed); // 原始反馈转速 (rad/s)，电机极性
    VofaSend();

    // 3. 发送
    xQueueOverwrite(shoot2cmd_queue_handle, &shoot_shoot2cmd_data); // 通过队列
}
