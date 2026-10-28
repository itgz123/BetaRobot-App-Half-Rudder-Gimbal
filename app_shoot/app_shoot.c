#include "app_shoot.h"
#include "app_shoot_def.h" // 发射整定参数（宏 + 驱动律说明），本文件只放控制逻辑
#include "app_cfg.h"
#include "app.h"
#include "app_cmd_shoot.h" // cmd2shoot / shoot2cmd 队列句柄与数据结构
#include "robot_def.h"     // gimbal限位/速度/加速度宏 + TRIGGER_STEP_RAD（每发机构转角）
//
#include "drv_djimotor_broadcast.h" // DJI 广播纯协议层
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
DRV_DJIMOTOR_BROADCAST_INSTANCE_DEF(friction_motor);    // 摩擦轮 M3508 (C620)
DRV_DJIMOTOR_BROADCAST_GROUP_DEF(friction_motor_group); // 摩擦轮广播组（一条总线一组）
DRVLIB_MOTOR_INSTANCE_DEF(friction_motor_ctrl);          // 摩擦轮速度环（三级 PID 核的中间环）
DRV_DJIMOTOR_BROADCAST_INSTANCE_DEF(trigger_motor);     // 拨弹 M3508 (C620)
DRV_DJIMOTOR_BROADCAST_GROUP_DEF(trigger_motor_group);  // 拨弹广播组（CAN_1，独立于摩擦轮组）
DRVLIB_MOTOR_INSTANCE_DEF(trigger_motor_ctrl);           // 拨弹 速度环（三级 PID 核的中环；无位置环）
// 通信
static cmd2shoot_data_t shoot_cmd2shoot_data; // cmd-shoot
static shoot2cmd_data_t shoot_shoot2cmd_data; // shoot-cmd
// 数据
static float friction_motor_setref = 0; // 摩擦轮目标转速 (rad/s)：使能时 = FRICTION_SPEED_FULL，失能时 0
static float trigger_friction_ff = 0;   // 拨盘摩擦前馈 (A)：库仑项 + **起步一两拍**叠加的静摩擦突破项，见 §静摩擦前馈

// 拨盘参考：单发/三发/连发都走 lib_traj_jerk 点到点轨迹，**终点必落在发数网格上**
LIB_TRAJ_INSTANCE_DEF(trigger_traj);     // 本段（一步 / 三发一程 / 连发一整段 / 收尾段）的预计算轨迹
static LibTraj_Output_s trigger_ref_out; // 上一拍参考输出（position/velocity/acceleration）
static float trigger_angle_ff = 0;       // 角度环(速度设定值域)前馈 = 参考速度；空闲恒 0
static float trigger_target_pos = 0;     // 本段轨迹终点 (rad, app 正向, 多圈)：绝对发数网格上的一格
static uint8_t trigger_planned = 0;      // 当前是否有有效轨迹在跑
static uint8_t trigger_active = 0;       // 上一拍是否在主动驱动机构（退出沿清一次 PID 并锁位）
// 连发收尾
static uint8_t trigger_burst = 0;    // 当前跑的是连发长轨迹（松手后需再走一条收尾轨迹才真正停）
static uint8_t trigger_settling = 0; // 正在走连发的收尾轨迹（终点为整数发边界、终点静止）
/* 上一拍是否判为"卡住"：用来抓**脱困沿**（卡住→动起来）清速度环积分，见 §卡住保护 */
static uint8_t trigger_stuck = 0;
/* "卡住且输出顶在硬限幅附近"已连续保持的时长 (ms)：真卡死要**持续**超阈值才算，见 §卡住保护 */
static float trigger_stuck_hold_ms = 0;
/* 发数网格（见 app_shoot_def.h §发数网格）：参考位置恒为
 *     trigger_grid_origin + trigger_ref_shot × TRIGGER_STEP_ADVANCE_RAD
 * 锚点 trigger_grid_origin 只在开机第一发规划时锚一次（假设那时机构停在某个整数位置），
 * **永不重锚**；trigger_ref_shot 就是"参考站在第几格"，每发规划只 **+= N**。
 * ⚠ 参考**只前进、不看实测**：实测位置只当轨迹起点用。第四版拿"实测的最近格点"当基准，
 *   机构少走超半格时参考会**后退一整发**（退弹），已推翻。 */
static float trigger_grid_origin = 0;
static int32_t trigger_ref_shot = 0;
static uint8_t trigger_grid_valid = 0;

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
    trigger_ev_step_done_e,   // 走完一发：三种模式统一为"本段轨迹 finished"（见 app_shoot.c 收尾判据）
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
    BSP_ASSERT_APP_CALL(DrvDJIMotorBroadcastRegister(&friction_motor));
    DrvDJIMotorBroadcastConfig_s friction_motor_cfg = {
        .group = &friction_motor_group,
        .can_e = CAN_2,
        .model = DRV_DJI_MODEL_M3508,
        .motor_id = 7,
        .torque_constant = 1, // 转矩常数 (Nm/A)
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvDJIMotorBroadcastConfig(&friction_motor, &friction_motor_cfg));

    // 控制核：速度环（ref = rad/s → PID → 电流）
    DrvlibMotor_Config_s friction_motor_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_SPEED, // 速度闭环
        /* 安装方向（2026-10-10 看实机定，原为正向）：摩擦轮"正转"是另一个物理旋向。
         * 反馈与输出**成对**取反（= 重定义正方向，闭环仍是负反馈）；只反一个会变成正反馈。 */
        .motor_direction = DRVLIB_MOTOR_DIRECTION_REVERSE,    // 输出方向
        .feedback_direction = DRVLIB_MOTOR_DIRECTION_REVERSE, // 反馈方向
        .position_mode = DRVLIB_MOTOR_POS_CONTINUOUS,         // 不用位置环（连续模式，不引入限幅）
        .angle_limit_max = 0,
        .angle_limit_min = 0,
        .position_span = 6.2831853f, // 单圈 2π（M3508 编码器跨度）
        .position_offset = 0,
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE, // 速度/电流低通
        /* 速度低通 20 ms → 5 ms（2026-10-11）：弹丸过轮的负载实测是 **~78 ms 的长浅凹陷**，20 ms
         * 一阶低通意味着环路要等 20~40 ms 才"看见"掉速、最该补电流的那一段全在滞后里。降到 5 ms
         * 后环路 5~10 ms 内响应；穿越处相位滞后（kp=0.2 对应 τ_cl≈40 ms、≈4 Hz）只有 ~7°，
         * 不动稳定性。代价是反馈噪声抬高（原始转速逐拍 ±5%），经 kp=0.2 折算成电流纹波 ≈±0.7 A，
         * 可接受。⚠ 观测通道 CH20/CH21 取的是 drv 原始量，本来就绕过低通，不受本值影响。 */
        .lpf_rc = 0.005f,
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
                         * τ_cl≈78 ms（≈12.8 rad/s 带宽），而速度 LPF 极点（现 5 ms → 200 rad/s）远在
                         * 环路带宽之外 —— 带宽瓶颈是 kp、不是滤波，故 kp 翻倍把 τ_cl 压到 ~40 ms，
                         * 抗扰动带宽约翻倍（这是"抗扰动"唯一的直接杠杆）。
                         * 上升段（t10-90≈280 ms）本来就顶在驱动限幅上、与 kp 无关；实测 ripple 仅
                         * 0.16 rad/s，反馈干净，翻倍后信噪比仍充足。 */
                        .kp = 0.2f, // 比例系数
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
    BSP_ASSERT_APP_CALL(DrvDJIMotorBroadcastRegister(&trigger_motor));
    DrvDJIMotorBroadcastConfig_s trigger_motor_cfg = {
        .group = &trigger_motor_group,
        .can_e = CAN_1,
        .model = DRV_DJI_MODEL_M3508,
        .motor_id = 1,
        .torque_constant = 1, // 转矩常数 (Nm/A)
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DrvDJIMotorBroadcastConfig(&trigger_motor, &trigger_motor_cfg));

    /* 控制核：**位置环 + 速度环**（ANGLE|SPEED），设定值来自 lib_traj_jerk 的规划轨迹：
     * 外环 ANGLE 把轨迹 position 当设定值、输出速度设定值；中环 SPEED 再输出电流。
     * 轨迹 velocity 经 angle_feedforward_ptr 前馈进外环（量纲 = 外环输出 = 速度设定值 rad/s，正好）。
     * 位置模式用 CONTINUOUS：轨迹是**多圈绝对角**，不能 WRAP、也不设限位。
     * 摩擦/加速度前馈按计划留到下一轮（speed/current 两路仍 DISABLE，speed 指针先挂着备接）。 */
    DrvlibMotor_Config_s trigger_motor_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED, // 位置环 + 速度环
        /* 安装方向（2026-10-10 看实机定）：拨盘"正转"是另一个物理旋向。与摩擦轮同一套 ——
         * 反馈与输出**成对**取反（= 重定义正方向，闭环仍是负反馈）；只反一个会变成正反馈。 */
        .motor_direction = DRVLIB_MOTOR_DIRECTION_REVERSE,    // 输出方向
        .feedback_direction = DRVLIB_MOTOR_DIRECTION_REVERSE, // 反馈方向
        .position_mode = DRVLIB_MOTOR_POS_CONTINUOUS,         // 多圈绝对角，不限幅不归一化
        .angle_limit_min = 0,
        .angle_limit_max = 0,
        .position_span = 6.2831853f, // 单圈 2π（M3508 编码器跨度），用于多圈累加
        .position_offset = 0,
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE, // 速度/电流低通
        /* 速度低通 TRIGGER_SPEED_LPF_RC（20 → 5 → 3 ms）：这是环路里最大的一处相位滞后，
         * 每次要提 TRIGGER_SPEED_KP 都得先从这里把相位还回来。取值与理由见宏定义处。 */
        .lpf_rc = TRIGGER_SPEED_LPF_RC,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_EXTERNAL, // 外环前馈 = 轨迹速度 (rad/s)
        .speed_feedforward_src = DRVLIB_MOTOR_FF_EXTERNAL, // 摩擦前馈（库仑 + 起步一两拍的突破项）
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .angle_feedforward_ptr = &trigger_angle_ff,
        .speed_feedforward_ptr = &trigger_friction_ff,
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
                        /* kp = TRIGGER_SPEED_KP，2026-10-11 由 0.08 提到 0.2（用户："把拨盘的
                         * 位置速度环调硬"）。旧值 0.08 的来历与代价（2026-10-10 开环力矩档实测）：
                         * 目标 20 rad/s 稳态是**持续极限环**，峰峰包络 35.4→8.4→…→8.3 —— 阶跃
                         * 暂态 2.6 s 内衰减到 ~7，之后 18 s 平在 7~8 不衰减（真极限环）。
                         * 关键：kp 0.2→0.08 让主频 13.2→6.5 Hz（正比于 kp，说明在环路穿越处），
                         * 但**幅值几乎不变** —— 幅值不随增益走、只有频率随增益走，是非线性环节
                         * （摩擦/量化/死区）撑着极限环的签名，不是纯相位裕度问题；当时选择保稳定、
                         * 弃刚度。现在闭环形态已不同（位置环在外、轨迹速度前馈在内），而收尾段的
                         * 问题是**电流上不去**（见 TRIGGER_ANGLE_KP 的实测账），刚度比那点猎振要紧；
                         * 相位损耗从速度 LPF 里还（TRIGGER_SPEED_LPF_RC 5→3 ms）。
                         * ⚠ 若日志里出现低频猎振（CH6 在目标附近等幅振荡）或高频啸叫，先降 kp
                         *   而不是降 LPF —— 见宏定义处的稳定上限估算。 */
                        .kp = TRIGGER_SPEED_KP, // 比例系数
                        /* ki 由 1.0 降到 0.3：积分器零点 ki/kp 落在极限环频率之下 —— 积分项在
                         * 振荡频段近似"纯 P"，不额外贡献滞后。现在是 0.3/0.2 = 1.5 rad/s。
                         * 稳态差本就不大（1.5~2.8%），且 DC 摩擦已由前馈接管，不需要大 ki；
                         * ki 大了还会让**空闲期顶着静摩擦**的积分绕得更凶（见 §发数网格）。 */
                        .ki = 0.3f,                      // 积分系数 (1/s)
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

/*---------------- 发数网格工具（参考自持，见 app_shoot_def.h §发数网格） ----------------
 * 参考位置 = trigger_grid_origin + trigger_ref_shot × TRIGGER_STEP_ADVANCE_RAD，
 * trigger_ref_shot 由发数累加、只增不减；实测位置不参与。 */
/* 整数发号 → 网格上的绝对位置 (rad) */
static float TriggerGridShotPos(int32_t n)
{
    return trigger_grid_origin + (float)n * TRIGGER_STEP_ADVANCE_RAD;
}

/* 不小于 pos 的最小格号（即 ceil）：给连发收尾找"前方那一格" */
static int32_t TriggerGridCeilShot(float pos)
{
    float x = (pos - trigger_grid_origin) / TRIGGER_STEP_ADVANCE_RAD;
    int32_t n = (int32_t)x; /* 向零截断：正数偏小、负数已 ≥ x */
    if ((float)n < x)
    {
        n += 1;
    }
    return n;
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
     * 机械方向（2026-10-10 看实机定，原为正向）：摩擦轮"正转"是另一个物理旋向 —— 由控制核
     * Config 的 motor_direction / feedback_direction **成对**声明（= 重定义正方向，闭环仍是负反馈）。
     * app 这里只传协议原始量、不再手写负号；⚠ 只反一个会变成正反馈。 */
    DrvDJIMotorBroadcastData_s fm = DrvDJIMotorBroadcastGetData(&friction_motor);
    float friction_out = 0.0f; // 本拍下发给摩擦轮的电流指令 (A)：失能时恒 0，供 VOFA CH22 观测
    if (shoot_cmd2shoot_data.enable)
    {
        friction_motor_setref = FRICTION_SPEED_FULL;
        DrvlibMotorFeedback_s friction_fb = {
            .position = fm.position,
            .speed = fm.speed,
            .current = fm.current,
            .timestamp_us = fm.timestamp_us,
        };
        /* app 拿到的 dt 是**毫秒**（见 bsp_app.h 的 APP_TASK_DEF 语义），控制核要秒 */
        friction_out = DrvlibMotorSetRef(&friction_motor_ctrl, friction_motor_setref, &friction_fb, dt * 0.001f);
        DrvDJIMotorBroadcastSetRef(&friction_motor, 0); // friction_out
    }
    else // 之后考虑刹车
    {
        friction_motor_setref = 0.0f;
        DrvlibMotorReset(&friction_motor_ctrl); // 清 PID（含积分）与 LPF 状态，不动 enable 标志
        DrvDJIMotorBroadcastSetRef(&friction_motor, 0.0f);
    }
    DrvDJIMotorBroadcastGroupSend(&friction_motor_group);

    // ---- 拨盘：拨弹状态机 + jerk 受限轨迹位置环 ----
    /* 机械方向（2026-10-10 看实机定）：拨盘"正转"是另一个物理旋向 —— 同样由控制核 Config 的
     * motor_direction / feedback_direction **成对**声明（只反一个会变成正反馈），app 只传协议原始量。
     * 轨迹位置/速度都在这个"app 正向"帧里，规划起点直接用控制核处理后的**多圈**位置 data.position
     * （已含方向与多圈累加）。 */
    DrvDJIMotorBroadcastData_s t = DrvDJIMotorBroadcastGetData(&trigger_motor);
    /* 反馈结构体 trigger_fb 在下面 SetRef 前才构造 —— #if CMD_DEBUG_CHASSIS_ONLY 下位置环
     * 整段旁路、用不到它，放这里会是个未使用变量。 */

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
            /* 规划本段：从**当前实测**位置/速度出发，终点 = **参考格号 + shots** 那一格。
             *   shots：单发 1、三发 3、连发 TRIGGER_BURST_SHOTS（长轨迹，松手后另走一条收尾轨迹）。
             *   vmax：连发取射频换算值（射频就是这么被定下来的）；单发/三发取轨迹上限
             *   （100 rad/s），它们只有 13.4 / 40.2 rad 的行程，实际由 amax 决定快慢。
             * 起点速度取**实测**：连发收尾后紧接着再来一发时机构还在动，不能让轨迹从 v=0 起步。 */
            float p0 = (float)trigger_motor_ctrl.data.position;
            float v0 = trigger_motor_ctrl.data.speed;
            int32_t shots = 1;
            float vmax = TRIGGER_TRAJ_VMAX;
            if (trigger_fsm_ctx.continuous)
            {
                if (burst_now)
                {
                    shots = (int32_t)TRIGGER_BURST_SHOTS;
                    vmax = TRIGGER_SPIN_SPEED;
                }
                else
                {
                    shots = 3;
                }
            }
            trigger_burst = burst_now;
            trigger_settling = 0;
            /* 网格锚点：**只在开机第一发规划时锚一次**（锚在当时的实测位置，当作格点 0），此后永不
             * 重锚。无绝对编码器，只能假设"第一发开始时机构停在某个格点上"。
             * 参考格号 += shots，目标 = 新格号那一格 —— **参考只前进、不看实测**（用户方案的原话：
             * 无论实测超前还是滞后，都规划到下一个整数位置）。实测只当轨迹起点 p0，于是"落后多少"
             * 自动变成"这一发多走多少"，天然补上，不需要任何网格追回。 */
            if (!trigger_grid_valid)
            {
                trigger_grid_origin = p0;
                trigger_ref_shot = 0;
                trigger_grid_valid = 1;
            }
            trigger_ref_shot += shots;
            trigger_target_pos = TriggerGridShotPos(trigger_ref_shot);

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
            /* 整段轨迹跑完。**这就是唯一的收尾判据** —— 规划时终点取的就是绝对网格上的一格
             * （最近格 + shots 格），必落在整数发网格上，所以 finished 判据最干净、没有浮点边界问题（曾用"参考
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
         * 收尾目标 = **参考格号**再往前的那一格（终点仍是整数格、终点静止），并把参考格号一起
         * 更新过去，空闲期就锁在这一格。取"当前参考位置 + TRIGGER_BURST_STOP_MARGIN"**向上取整**：
         * 向上取整保证把当前这发打完（停在半发位置会让下一发的相位全错），余量保证终点恒远在
         * **前方** —— 否则 lib_traj_jerk 会因"行程短于减速距离"判不可行、参考冻在高速上。 */
        drive = 1;
        if (!trigger_settling)
        {
            trigger_ref_shot = TriggerGridCeilShot(trigger_ref_out.position + TRIGGER_BURST_STOP_MARGIN * TRIGGER_STEP_ADVANCE_RAD);
            trigger_target_pos = TriggerGridShotPos(trigger_ref_shot);
            float p0 = (float)trigger_motor_ctrl.data.position;
            float v0 = trigger_motor_ctrl.data.speed;

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
                /* 停稳。本拍仍用终点参考驱动（drive 保持 1），下一拍才落回"锁最近格点"，
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
         * 不"主动走轨迹"；设定值锁在最近的发数格点上（见下面 ref 那一段），位置环只做小幅
         * 相位回位（可能需要倒转）；还没有网格时退回锁实测。 */
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

    /* ---------------- 摩擦前馈（见 app_shoot_def.h §拨盘摩擦前馈 / §静摩擦前馈） ----------------
     * 库仑(动摩擦)项：只要有速度指令就给，顶住运行中那份摩擦。
     * 静摩擦突破项：**只在原始转速≈0 时给**（绝对带宽）。它是"一脚把机构踢动"的脉冲，
     *   原设计就写明"多给一拍就是一次加速冲击"，故判据必须窄到只覆盖起步那一两拍。
     *   ⚠ **故意不复用 §卡住保护 的相对判据**：相对判据在整个带载爬行段（~150 ms）都为真，
     *   用它给突破前馈就变成 6 A 的**持续**推力 —— 实测 CH19 在收尾段通断 7.9 次/发、每次摆 6 A，
     *   等于往环路里灌一个电流方波激励（且那时机构已在 25 rad/s 上爬，这一脚本来也帮不上忙）。
     * 判据与 §卡住保护 一样取 drv 的**原始**转速、不走速度低通（低通 rc=20 ms 会把起步那一两拍
     *   的"没动"抹掉，判据迟 2~3 拍才成立）。
     * 方向取**目标速度**符号、不取实测转速：静摩擦时实测恒≈0，用它定符号会在 0 附近抖。
     * 本值在 SetRef 之前赋值 —— 控制核在级联里读 `speed_feedforward_ptr`，晚赋值就晚一拍。 */
    float trigger_ff_mag = 0.0f;
    if (drive && fabsf(trigger_angle_ff) > TRIGGER_FRICTION_FF_EPS)
    {
        trigger_ff_mag = TRIGGER_FRICTION_FF;
        if (fabsf(t.speed) < TRIGGER_FRICTION_FF_STATIC_BAND)
        {
            trigger_ff_mag += TRIGGER_FRICTION_FF_STATIC; // 起步那一两拍的一脚
        }
    }
    trigger_friction_ff = (trigger_angle_ff >= 0.0f) ? trigger_ff_mag : -trigger_ff_mag;

    float trigger_ref;
#if CMD_DEBUG_CHASSIS_ONLY
    /* 调试底盘（见 app_cfg.h）：拨盘也只发 0 力矩 —— 位置环整段旁路。
     * 本文件不读 cmd 的 enable（拨盘只看 fire_mode），光靠 cmd 下发 fire_mode=off 仍会锁格点闭环，
     * 故这道门控必须在这里。不调控制核也就没有积分残留（本宏下驱动核从头到尾不被使用），无需 Reset。
     * trigger_ref 在调试下没有参考可言，取实测位置占位 —— VOFA CH1/CH3 仍照发（CH3 恒 0）。 */
    trigger_ref = (float)trigger_motor_ctrl.data.position;
    float trigger_out = 0.0f;
#else
    /* 驱动时用上面的参考位置；空闲时设定值锁在**参考自己所在的那一格**上（不是实测位置）——
     * 这是相位漂移的第二道闸（见 §发数网格）：收尾超调 / 带载滞后留下的偏差，会在空闲期被位置环
     * 慢慢拉回格点，而不是像旧版"锁实测"那样把偏差固化成下一发的起点。
     * ⚠ 基准是 **trigger_ref_shot** 而不是"实测的最近格点"：参考只前进、不看实测。第四版取实测的
     *   最近格点，机构少走超半格时参考会**后退一整发**（退弹），已推翻。
     * 还没有网格（开机首帧、或从未发过弹）时退回"锁实测"，与旧版一致。 */
    if (drive)
    {
        trigger_ref = trigger_ref_out.position;
    }
    else if (trigger_grid_valid)
    {
        trigger_ref = TriggerGridShotPos(trigger_ref_shot);
    }
    else
    {
        trigger_ref = (float)trigger_motor_ctrl.data.position;
    }
    DrvlibMotorFeedback_s trigger_fb = {
        .position = t.position,
        .speed = t.speed,
        .current = t.current,
        .timestamp_us = t.timestamp_us,
    };
    float trigger_out = DrvlibMotorSetRef(&trigger_motor_ctrl, trigger_ref, &trigger_fb, trigger_dt_s);
#endif

    /* ---------------- 卡住判据（"要求它动、但机构没跟住"，口径见 app_shoot_def.h §卡住保护） ----------------
     *   ① 期望要动：|前馈速度| > TRIGGER_STUCK_CMD_BAND（把轨迹的起停段排除在外）
     *   ② 实际没跟住：|**原始**转速| < 绝对下限 + 比例 × |前馈速度|
     * ②取原始转速的理由同上（低通是延迟）。判据在 SetRef **之后**算：切断那一路要看本拍输出。 */
    uint8_t stuck_now = (uint8_t)(drive && fb_valid && (fabsf(trigger_angle_ff) > TRIGGER_STUCK_CMD_BAND) &&
                                  (fabsf(t.speed) < TRIGGER_STUCK_SPEED_BAND + TRIGGER_STUCK_SPEED_RATIO * fabsf(trigger_angle_ff)));

    /* ---------------- 卡住保护（真卡死切断）与脱困清积分（见 app_shoot_def.h §卡住保护） ----------------
     * 卡住期间积分器**照常累积**（本方案的前提，别在这里把 ki 置 0），两个出口：
     *   1) 输出**持续**顶在 TRIGGER_STUCK_CUT_A 以上 TRIGGER_STUCK_CUT_HOLD_MS → 判真卡死：
     *      本拍输出清零 + 清积分，下一拍环路自然重来。加"持续"门槛是因为相对判据让"卡住"在带载
     *      爬行段全程为真（~150 ms），单看瞬时电流会把正常推弹的尖峰也切掉。
     *   2) 卡住 → 不卡住的**脱困沿** → 清积分，消掉"爬到突破值"的那股储能（否则脱困瞬间它变成
     *      一股净加速电流把机构顶飞）。 */
    uint8_t stuck_cut = 0; // 本拍是否真的切断了输出（VOFA CH15 用，须在清零 trigger_out **之前**判定）
    if (stuck_now && (fabsf(trigger_out) > TRIGGER_STUCK_CUT_A))
    {
        trigger_stuck_hold_ms += dt; // dt 是毫秒，与 TRIGGER_STUCK_CUT_HOLD_MS 同量纲
        if (trigger_stuck_hold_ms >= TRIGGER_STUCK_CUT_HOLD_MS)
        {
            stuck_cut = 1;
            trigger_stuck_hold_ms = 0.0f;
            trigger_out = 0.0f;                                            // 真卡死：本拍切断，下拍环路重来
            trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED].i_out = 0.0f; // 这份储能不脱困也用不上了，一并清掉
        }
    }
    else
    {
        trigger_stuck_hold_ms = 0.0f; // 离开"顶住"状态就复位 —— 本判据要求**连续**保持
    }
    if (trigger_stuck && !stuck_now)
    {
        trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED].i_out = 0.0f; // 脱困沿：清掉"爬到突破值"的储能
    }
    trigger_stuck = stuck_now; // stuck_now 已含 drive（非驱动拍恒 0），脱困沿只在一条轨迹内部成立

    float trigger_cmd = trigger_out; // 方向已由控制核的 motor_direction 修正，这里不再取负

    DrvDJIMotorBroadcastSetRef(&trigger_motor, 0); // trigger_cmd
    DrvDJIMotorBroadcastGroupSend(&trigger_motor_group);

    /* 调试：拨盘**参考位置环**观测通道（本块自己发帧；app_gimbal 的 IMU 那路 VOFA 已关）。
     *   CH1  参考位置 (rad，app 正向，多圈)          —— 位置环给定，三种模式都是同一条 lib_traj 轨迹
     *   CH2  实测位置 (rad，app 正向，多圈)          —— data.position，与 CH1 之差即跟踪误差
     *   CH3  位置误差 = CH1 - CH2 (rad)
     *   CH4  实下发力矩指令 (Nm) = SetRef 返回值（方向已含在控制核内）
     *   CH5  参考速度 (rad/s) —— 同时就是**外环速度前馈**。单发一步 13.4 rad、加/减速各需
     *        3.11 rad < 13.4，故够得到 vmax，是"加速-巡航-减速"的梯形包络；连发长轨迹除首尾
     *        两段外恒为巡航值 TRIGGER_SPIN_SPEED（= 射频换算值）
     *   CH6  实测转速 (rad/s，app 正向，低通)        —— data.speed
     *   CH7  参考加速度 (rad/s²)：轨迹加速度（7 段 S 曲线的加/减速相位；巡航段为 0）
     *   CH8  实测电流 (A，app 正向)：本电机 torque_constant=1，数值上 ≈ 力矩
     *   CH9  本拍 dt (ms)：确认控制周期；有跳变说明任务被拖或 CAN 掉帧
     *   CH10 状态机状态 (0关/1单发/2三发/3连发/4停止)
     *   CH11 驱动条件 enable && fire_mode!=off（等价于 enable 且 ch7 闭合）(0/1)
     *   CH12 原始反馈转速 (rad/s，**电机极性**，未取反)：用于核对方向约定
     * ---- CH13~CH19：带负载整定用的控制核内部量（2026-10-11 加，调摩擦前馈/积分时看） ----
     *   ⚠ **极性分两套，别混**：控制核内部量（CH13/14/16/17/18）都在 **app 正向域**，
     *     只有 CH4 是**电机域**（控制核在返回前乘了 motor_direction）。故未饱和时
     *         CH4 = −(CH13 + CH14 + CH19) （速度环 P + I + 摩擦前馈；d 项恒 0）
     *         CH16 = CH17 + CH5           （位置环 ki=0，输出 = P + 前馈；触限时不等）
     *     |CH13+CH14| > |CH4| 就是速度环**输出限幅饱和**（CH4 顶在 ±TRIGGER_I_MAX）。
     *   CH13 速度环 P 项 p_out (A，app 正向)
     *   CH14 速度环 I 项 i_out (A，app 正向) —— **摩擦前馈是否补到位的唯一判据**：补到位时它应
     *        在 0 附近正负晃；系统性压在某一侧 = 前馈偏了。本通道也是"卡住保护清积分"的观测点
     *        （脱困沿/切断拍会看到它被拉回 0）。
     *   CH15 卡住保护状态：0=正常 / 1=卡住但输出未达阈值或未持续够时长（还在爬） /
     *        2=卡住且**已持续**超过 TRIGGER_STUCK_CUT_HOLD_MS 并切断输出（见 §卡住保护）
     *        —— 现在带载推弹全程都是 1（相对判据），看到 2 才是真卡死
     *   CH16 位置环输出 = 速度环设定值 (rad/s，app 正向)：顶到 ±TRIGGER_ANGLE_VMAX = 位置环饱和
     *   CH17 位置环 P 项 p_out (rad/s，app 正向，**未限幅**)：触限时它比 CH16 更能看出"想跑多快"
     *   CH18 速度环误差 e = CH16 − CH6 (rad/s，app 正向)：|e| 落在哪一档决定变速积分的系数
     *        （≤TRIGGER_VARINT_B=40 全积分 / 40~140 衰减 / >140 冻结），带负载掉速时看它
     *   CH19 速度环前馈 = trigger_friction_ff (A)：库仑项 (TRIGGER_FRICTION_FF) 恒在，
     *        原始转速≈0 的起步拍再叠加 TRIGGER_FRICTION_FF_STATIC —— 与 CH14 对照看是否补到位
     *        （补到位则 CH14 趋 0），以及看突破项有没有在正常推弹/收尾段误触发
     * ---- CH20~CH25：摩擦轮（**判断弹丸有没有真的过轮、什么时候过轮**，2026-10-11 加）----
     * 判弹思路：摩擦轮空转恒速，弹丸被两轮夹着挤过去的那 2~5 ms 会**掉速**，速度环随即**顶电流**
     *   补回来 —— 所以"过弹"的签名是 CH20 掉一个坑、CH21/CH22 冒一个尖、CH23（积分项）随后来一段
     *   恢复。拨盘侧只能看到"走了多少角度"，弹丸到底出没出只能靠这一组。
     * ⚠ **必须用 drv 原始量（未低通）**：摩擦轮反馈在控制核里过 5 ms 低通、且乘了
     *   feedback_direction，走 `data.speed` 已是低通后的值。故 CH20/CH21 取 `fm` 原始量并按 CH8
     *   的约定取负到 app 正向；CH22~CH24 是控制核内部量，本来就是 app 正向，不用取负。
     *   `DrvDJIMotorBroadcastGetData()` 返回的**就是**协议原始量（drv 明确不做滤波/方向/多圈）：
     *   CH20 = int16 rpm × 2π/60、CH21 = int16 raw × a_per_lsb —— 不需要往 drv 里再加通道。
     * ⚠ 采样 500 Hz。实测弹丸过轮是 **~78 ms 的长浅凹陷**（转速只掉 ~3%、电流 0.4→1.2 A），
     *   不是短促深坑 —— 判弹优先看 CH21/CH22 的平滑电流凸起与 CH23 的积分恢复，CH20 作辅助。
     *   CH20 摩擦轮实测转速 (rad/s, app 正向, **未低通**)      CH21 摩擦轮实测电流 (A, app 正向, **未低通**)
     *   CH22 摩擦轮速度环输出 = 实下发 (A, app 正向)            CH23 摩擦轮速度环 I 项 (A)
     *   CH24 摩擦轮速度误差 (rad/s)                             CH25 摩擦轮使能且在线 (0/1)
     *        —— CH25 用来区分"过弹时没有尖峰"和"摩擦轮根本没转"（enable 断开或 CAN 掉线） */
    VofaSetChannel(1, trigger_ref);
    VofaSetChannel(2, (float)trigger_motor_ctrl.data.position);
    VofaSetChannel(3, trigger_ref - (float)trigger_motor_ctrl.data.position);
    VofaSetChannel(4, trigger_cmd);
    VofaSetChannel(5, trigger_angle_ff);
    VofaSetChannel(6, trigger_motor_ctrl.data.speed);
    VofaSetChannel(7, drive ? trigger_ref_out.acceleration : 0.0f);
    VofaSetChannel(8, -t.current); // 实测电流 (A)，app 正向：drv 原始量无方向，显式取负（未 LPF，保饱和判据）
    VofaSetChannel(9, dt);         // 本拍 dt (ms)：app 拿到的 dt 就是毫秒
    VofaSetChannel(10, (float)LibFsmTableCurrent(&trigger_fsm));
    VofaSetChannel(11, (float)(shoot_cmd2shoot_data.enable && (shoot_cmd2shoot_data.fire_mode != fire_mode_off_e)));
    VofaSetChannel(12, t.speed); // 原始反馈转速 (rad/s)，电机极性
    /* CH13~CH19：控制核内部量（app 正向域，见上面极性说明）。空闲拍仍照发，便于看停机后的残值 */
    VofaSetChannel(13, trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED].p_out);
    VofaSetChannel(14, trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED].i_out);
    VofaSetChannel(15, (float)(stuck_now ? (stuck_cut ? 2 : 1) : 0));
    VofaSetChannel(16, trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_ANGLE].output);
    VofaSetChannel(17, trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_ANGLE].p_out);
    VofaSetChannel(18, trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED].error);
    VofaSetChannel(19, trigger_friction_ff);
    /* CH20~CH25：摩擦轮（判弹丸过轮，见上面通道表）。空闲拍仍照发，便于看停机后的残值 */
    VofaSetChannel(20, -fm.speed);                                                      // 实测转速 (rad/s, app 正向, 未低通)
    VofaSetChannel(21, -fm.current);                                                    // 实测电流 (A, app 正向, 未低通)
    VofaSetChannel(22, friction_out);                                                   // 速度环输出 = 实下发 (A)
    VofaSetChannel(23, friction_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED].i_out);        // 速度环 I 项 (A)
    VofaSetChannel(24, friction_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED].error);        // 速度误差 (rad/s)
    VofaSetChannel(25, (float)(shoot_cmd2shoot_data.enable && (fm.timestamp_us != 0))); // 使能且在线
    VofaSend();

    // 3. 发送
    xQueueOverwrite(shoot2cmd_queue_handle, &shoot_shoot2cmd_data); // 通过队列
}
