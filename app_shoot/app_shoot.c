#include "app_shoot.h"
#include "app_cfg.h"
#include "app.h"
#include "app_cmd_shoot.h" // cmd2shoot / shoot2cmd 队列句柄与数据结构
#include "robot_def.h"     // gimbal限位/速度/加速度宏
//
#include "drvs_djimotor_broadcast.h" // DJI 广播纯协议层
#include "drvlib_motor.h"            // 三级级联 PID 控制核
#include "lib_fsm_table.h"           // 表驱动状态机（拨弹状态机）
#include "drv_vofa.h"
//
#include "bsp_sys_status.h"
#include "bsp_freertos.h"
//
#include <math.h>    // fabsf（状态机的行程累计取幅值）
#include <stdbool.h> // 状态机守卫

// 拨弹控制参数（占位，实机整定；机械参数见 robot_def.h 的 TRIGGER_STEP_RAD）
#define TRIGGER_V_MAX 20.0f  // 拨盘速度上限 (rad/s)：位置环输出限幅
#define TRIGGER_A_MAX 200.0f // 轨迹最大加/减速度 (rad/s²)（lib_traj 已从本文件摘除，暂未引用）
#define TRIGGER_I_MAX 20.0f  // 速度环输出(电流)上限 (A)，同时约束加速度
/* 拨盘位置反馈是单圈值 [0,2π)，行程累计要处理回绕：逐拍增量 |Δ| 超过半圈即认为跨界 */
#define TRIGGER_POS_WRAP_FULL 6.2831853f
#define TRIGGER_POS_WRAP_HALF (TRIGGER_POS_WRAP_FULL * 0.5f)

/*-------------------------------- 电机模式 --------------------------------
 * 比赛形态：云台立起 → 摩擦轮启动后全程不停，发弹只需拨盘动作；两个电机互不联动。
 * 本阶段：
 *   - 摩擦轮：只有"使能/失能"两态（enable 由 cmd 算，见 app_cmd_shoot.h）。使能跑速度环、
 *     目标就是满速（见 FRICTION_SPEED_FULL）；失能**不跑控制核**，直接下发 0 A 电流帧，
 *     并清一次 PID —— C620 保持使能、零力矩自由滑行，不掉线也不报错。
 *   - 拨盘  ：只跑状态机（见 AppShootRun），位置/速度环与轨迹规划都未接入，固定给 0。
 * 两者的 PID 配置与实例都留在 AppShootInit 里，接回时只需恢复"从控制核取输出"那几行。 */

/* 摩擦轮"全速"目标转速 (rad/s)：本阶段弹速/射频来源都没接线，使能就只有"满速"一档，
 * 取规则弹速上限对应的电机转速（半径/传动比见 robot_def.h，量与反馈同轴）。
 * 注意：该值是按 BULLET_SPEED_MAX 反推的，若实机顶在 FRICTION_I_MAX 不回落，说明它超过
 * 了可达转速（速度环饱和、持续大电流），往下调即可 —— 唯一用途就是"起转到满"。 */
#define FRICTION_SPEED_FULL BULLET_SPEED_TO_MOTOR_RADPS(BULLET_SPEED_MAX)

/* 拨盘摩擦前馈（库仑摩擦补偿）：拨盘摩擦大，全靠 PI 现爬会有启动迟滞和低速稳态差。
 * 前馈按运动方向直接给出"克服摩擦所需"的那份力矩，PI 只补模型误差和动态误差。
 * 加在速度环（力矩域）—— 位置环输出是速度、量纲不符；速度环输出才是力矩，
 * lib_pid 里 ff 与 PI 相加后才统一限幅（见 lib_pid.c 输出段）。
 * 方向取目标速度符号（不是实际速度：静摩擦时实际速度≈0，取它会让符号抖），并留死区。
 *
 * 本宏只给**动摩擦**（运行中要顶住的那份）。静摩擦（起停那一下的突破）是另一项、
 * 单独由 TRIGGER_FRICTION_FF_STATIC 负责 —— 二者量级差 6 倍，混在一起用就两头不讨好：
 *   动摩擦（运行中要顶住的）≈ 0.2 A → 本宏；
 *   静摩擦（起停瞬间那一下突破）≈ 1.3 A → 见 TRIGGER_FRICTION_FF_STATIC（实测值）。
 * 一度按"约 0.45 A"估静摩擦，09:04 全行程日志实测是 1.06~1.26 A，那个估计偏低一倍多。
 * 也一度把动摩擦取到 0.5 A（照搬静摩擦）：运动时前馈比动摩擦大一倍多，逼得积分器反向
 * 抵消到 -0.24~-0.37 A 才能让输出回到 0.2 A 的维持值 —— 前馈不是在帮忙，是在制造
 * 一个恒定的力矩偏置让积分器去追。
 *
 * 2026-10-10 第二轮：前馈改成**两项**（库仑 + 粘性）。08:48 日志拟合出维持电流随转速
 * 线性上升 I(ω) = 0.1895 + 0.8446e-3·ω [A]（R²=0.85），据此给"截距 + 斜率"。
 *
 * 2026-10-10 第三轮：**斜率项撤掉，回到常数，但值改成 0.24 A。**
 * 那条斜率是"两个转速簇"拟合出的假象：08:48 日志只有 42.9 和 100 两个转速，
 * 而 100 rad/s 的台段在时间轴上更靠后 —— 转速与时间共线，温度漂移被算成了速度依赖。
 * 08:56 日志多了 30.5/57.6 两个点，用稳态平衡式 out = ff + p + i 反推"真实摩擦需求"
 * （ff 与 i 取自同一拍，漂移对所消）：
 *       30.5 rad/s → 0.239 A    57.6 rad/s → 0.218 A    100 rad/s → 0.259 A
 * 线性模型最大偏差 0.019 A、常数模型 0.021 A —— 分不出来。数据不支持斜率就不硬凑，
 * 取常数 0.24 A（三点均值 0.2387）。判据看稳态积分项：补到位应是在 0 附近正负晃，
 * 而不是系统性地压在某一侧。
 *
 * 2026-10-10 第四轮：0.24 A **验证通过，不动**。09:04 全行程日志 20 个中高速档
 * （23~82 rad/s）的稳态平衡式 out = ff + p + i 里，i_ss 的中位数是 -0.015 A（6% 的 F_c），
 * 且随转速单调趋近 0（14.7 rad/s 时 -0.070，74 rad/s 以上 -0.01~+0.01）——
 * 即"前馈已经补到位、积分器基本无事可做"，正是想要的判据形态。
 * 反推的**真实需求** out_ss 从 0.170 A（14.7 rad/s）缓升到 0.235~0.275 A（74~100 rad/s），
 * 看着像有斜率，但与第三轮否掉的那条斜率同样受"转速-时间共线"污染，且幅度只有 F_c 的 ±15%，
 * 不值得为它引入一个假的粘性项。保持常数，把 TRIGGER_FRICTION_FF_VISC 留作调试旋钮。
 *
 * 2026-10-10：拨盘电机通路整段停用（lib_traj 摘除 + 电机固定给 0），本组前馈量暂不在
 * AppShootRun 里计算 —— 参数与 speed_feedforward 指针都留着，接回电机通路时直接用。 */
#define TRIGGER_FRICTION_FF 0.24f     // 库仑(动摩擦)前馈幅值 (A)：三点反推均值，30~100 rad/s 通用
#define TRIGGER_FRICTION_FF_VISC 0.0f // 粘性分量 (A/(rad/s))：数据不支持非零斜率，留作调试旋钮
#define TRIGGER_FRICTION_FF_EPS 0.5f  // 前馈死区 (rad/s)：|目标速度| 小于此值不给前馈

/*-------------------------------- 静摩擦（突破）前馈 --------------------------------
 * 09:04 日志（0~100 全速度多档阶跃）暴露的真问题：低速档 (≲15 rad/s) 是**粘滑极限环**，
 * 不是"补偿量不够"。现象（ref=3.32 档，t=104~108 s）：转速长时间死在 0.00，积分器一路
 * 爬到 1.0 A，然后突然脱困冲到 13~20 rad/s（设定值的 4~6 倍），再卡死、再冲，周期约 1 s。
 *
 * 突破阈值量得很干净：取每次"卡→动"跳变前一拍的输出，9 次独立脱困落在
 *   1.06 / 1.09 / 1.11 / 1.08 / 1.15 / 1.21 / 1.09 / 1.19 / 1.26 A
 * （同拍实测电流 1.08~1.25 A 同步，不是噪声/指令假的）——**静摩擦突破要 ≈1.25 A，
 * 运行中只要 ≈0.2 A，差 6 倍**。哪个档卡、卡多久，用"积分器爬到突破值要多久"
 * t ≈ F_s/(ki·|e|) 全能对上（ki=0.3，F_s≈1.25）：
 *   ref 14.67 → kp·|e| = 1.17 A 已接近突破值      → 卡 0.4%
 *   ref  5.93 → t ≈ 1.25/(0.3×5.93) = 0.70 s      → 卡 25~29%
 *   ref  3.32 → t ≈ 1.25/(0.3×3.32) = 1.25 s      → 卡 54%
 *   ref  0.64 → t ≈ 6.5 s > 该档时长 1.6 s        → 卡 99.5%，整档一步没动
 * 而脱困后冲多高，由积分器的**退**饱和速度 ki·|e|（≈5 A/s，退掉 0.7 A 要 0.14 s）决定
 * ——正是那 0.15 s 的 1.3 A 过流把转速顶到 20。所以是**积分器在储能**，不是前馈给少了；
 * 单纯加大 TRIGGER_FRICTION_FF 治不了（那样每个高速工作点都要被积分器反向抵消 1 A）。
 *
 * 修法是两项，缺一不可：
 *   1) 卡住时（|ω_实测| < 本带宽）把前馈抬到突破值 → 一个控制周期就脱困，不用等积分器爬 1.25 s；
 *   2) 一旦动起来立刻收回到动摩擦值 → 突破值是动摩擦的 6 倍，多给一拍就是一次加速冲击。
 * 判"卡住"必须用**原始**反馈转速，不能走 drvlib 的速度低通（rc=0.02 → 20 ms 延迟，等于
 * 多给 20 ms 的 6 倍过流，自己就能造出一个小极限环）。控制周期 2 ms，收回延迟 ≤ 2 ms，
 * 按实测减速斜率（~73 rad/s²/A）折算过冲出约 0.2 rad/s，可忽略。
 * 残留风险：机构真被卡死（卡弹）时，本项会把电流顶在 1.35 A 等积分器继续爬到 20 A 限幅，
 * 发热与今天一致，未变差。
 *
 * 2026-10-10 验证（09:16 日志，新固件首份）：判据工作完全符合设计 —— CH10==1.35 共 69 个
 * 样本、33 次事件（每个阶跃起始一次），时长 2~22 ms（多数 2~4 ms，即 1~2 拍就收回），
 * 触发时 |原始转速| ≤ 0.09 rad/s（深在带宽内）。反查"该触发却没触发"的样本：0 个。
 * 另有 15 个样本 FF=0.24 而**低通后**转速 <0.4、原始转速 0.52~1.36 —— 若判据用低通转速，
 * 这 15 个就是误触发：这正是判据必须取原始转速的实证。
 * 但同一份日志也暴露：实测区间只到 ref 22.9（旋钮 −0.542），而突破项只在
 * |ref| ≲ F_s/kp = 15 rad/s 才有意义 —— 高速档的 P 项 (0.08×92 = 7.4 A) 本来就远超
 * 突破值，这一脚是白送的，故加了"P 项顶不到才补"的门槛（见 AppShootRun）。
 *
 * 2026-10-10 复测（09:28 日志，全行程连续扫：ref 2.04~100）：低速段终于有数据了。
 * 极限环周期从 ~1 s 缩到 ~110 ms、峰值从 20 缩到 ~8，**但没根治**：
 *   ref 2~4  档：中位 1.15（掉 62%）、峰峰 27.4、|meas|<0.4 占 17.5%、I 中位 +0.255
 *   ref 4~6  档：中位 4.24（掉 15%）、峰峰 24.4、I 中位 +0.116
 *   ref 6~8  档：中位 7.01（已跟随）、峰峰 20.0、I 中位 -0.025
 *   ref ≥8   档：中位跟随在 ±2% 内，I 中位 -0.06~-0.01，out 中位 0.19~0.25 ≈ F_c
 * 关键读数：低速段 **I 中位 +0.255 A ≈ F_c(0.24)** —— 积分器又变回"摩擦项"了。原因是
 * 突破项只在 |原始转速|<带宽 时给，转速一过 0.4 就收回 0.24，中间那段（转速 0.4~ref）
 * 只能靠积分器补，于是它把 0.2~0.3 A 偏置储起来；脱困后这份偏置就是 net 加速电流，
 * 把平衡点推到 ω* = ref + I/kp = 2.6 + 3.3 = 5.9，峰到 7.9（ref 的 3 倍）。
 * 结论：**突破项 +"卡住时清积分"必须成对**，单独上突破项只能把 1 s 的粘滑换成 110 ms
 * 的小极限环。故 AppShootRun 里卡住拍把 speed 环 ki 置 0、i_out 清 0（见那里的注释）。 */
#define TRIGGER_FRICTION_FF_STATIC 1.35f     // 静摩擦突破前馈 (A)：实测阈值 1.06~1.26，留 ~8% 余量
#define TRIGGER_FRICTION_FF_STATIC_BAND 0.4f // 判"卡住"的实测转速带宽 (rad/s)：≈4 rpm，比量化(1 rpm)高 3 计数

/* 拨盘速度环的变速积分门限，算法与摩擦轮同一套（语义见 lib_pid.c
 * f_Changing_Integration_Rate），只是量纲换成拨盘：拨盘速度误差量级是 TRIGGER_V_MAX=20，
 * 不是摩擦轮的 880。B = 5 → 稳态/小误差全积分；A+B = 20 当初是照 TRIGGER_V_MAX=20 定的
 * → 满幅速度阶跃的起始误差直接落进冻结区，防超调。占位值，实机整定。
 *
 * 注意（2026-10-10）：A+B=20 只覆盖到 TRIGGER_V_MAX 那一档，更大的速度阶跃起始误差直接
 * 落进冻结区 —— 目标 100 rad/s 时误差要从 100 掉到 20 才解冻（转速到 80），误差 5 才全积分。
 * 稳态不受影响（稳态 |e|<B 永远全积分，这就是两个工作点稳态差都 <0.2% 的原因），
 * 但**暂态形状随指令幅值变**：小指令（<20）积分全程参与、大指令只参与最后一段。
 * 后续若"到达过程的形态"对不上，这里是首要可疑点。 */
#define TRIGGER_VARINT_B 5.0f  // 变速积分 B：全积分带宽 (rad/s)
#define TRIGGER_VARINT_A 15.0f // 变速积分 A：衰减宽度 (rad/s)；旧值对齐 V_MAX=20，调试上限已到 100

/* 摩擦轮速度环电流上限 (A)：与 M3508 驱动侧限幅 (raw_max 16384 ↔ 20 A) 取齐，
 * 使 PID 的限幅与真限幅合拍（否则限幅外那段全是积分饱和区，见下）。
 * 整定数据（2026-10-09，空载阶跃 + 纸板加载）：稳态维持电流 ≈ 0.15 + 1.05e-3·ω A，
 * 满载 767 rad/s 仅 1.1 A；纸板加载额外 +5.5 A。积分上限取硬上限的一半，
 * 给比例项留出权限：积分单独顶到 10 A 时，比例仍有 100 rad/s 误差的动态余量，
 * 且干扰消失后能以 ki·|e| 的速度退出（20 A 会让退出太慢）。 */
#define FRICTION_I_MAX 20.0f   // 速度环输出(电流)硬上限 (A)
#define FRICTION_I_LIMIT 10.0f // 积分项上限 (A)，见上

/* 变速积分（积分分离）误差门限 (rad/s)，语义见 lib_pid.c f_Changing_Integration_Rate：
 *   |e| ≤ B           → 全积分（系数 1）
 *   B < |e| ≤ A+B     → 系数线性衰减 (A+B-|e|)/A
 *   |e| > A+B         → 积分冻结（i_term=0，仅当 e·i_out>0 时判，即只在会积饱和的方向上冻结）
 * 取值依据（2026-10-09 断后加载实测，本环 kp=0.2、out_max=20A）：
 *   手压负载把误差顶到 68~153 rad/s（掉速 8.2%~17.4%）。旧门限 B=20/A=80（A+B=100）让 153
 *   落进冻结区、68 只剩 0.4 系数——等于"最需要积分顶负载时把积分掐掉"：加载 3.17 s 内电流
 *   只到 13.45 A（离 20 A 限幅还有余量），转速却稳在 -17%，说明瓶颈是环路刚度不是驱动能力。
 *   本环抗扰动优先、过冲无所谓，故把门限抬到让整个扰动区间全积分：
 *   B   = 160 → 覆盖实测扰动峰值，|e|≤160 全额积分（原来 68 剩 0.4、153 直接冻结）。
 *   A+B = 400 → 仍远低于启动阶跃的 880 rad/s，上升饱和段照旧冻结，防超调的作用保留。
 * 效果（2026-10-09 实测，旧门限）：过冲 5~8%→1.6%、整定 700→400 ms。大误差（阶跃上升的电流
 *       饱和段 |e| 长时间 >100）积分不参与 → 冲起来的积分不再需要靠保守的 ki 去防；去饱和后
 *       系数平滑接管。注意：它不解决静止段量化偏置的慢爬（那个误差 <B、落在全积分区）。 */
#define FRICTION_VARINT_B 160.0f // 变速积分 B：全积分带宽 (rad/s)，≥ 实测扰动误差
#define FRICTION_VARINT_A 240.0f // 变速积分 A：衰减宽度 (rad/s)，A+B=400 ≪ 启动阶跃 880

/* 弹速 (m/s) → 摩擦轮电机转速 (rad/s)：v = ω·r，故 ω = v/r，再乘电机:摩擦轮传动比。
 * 半径/传动比都是 robot_def.h 里的机械量，改机械不动这里。
 * 本阶段无调用点：摩擦轮目标速度的来源（射频/裁判弹速）尚未接线。 */
#define BULLET_SPEED_TO_MOTOR_RADPS(v) ((v) / FRICTION_WHEEL_RADIUS * FRICTION_REDUCTION)

// 变量
// 实例
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(friction_motor);    // 摩擦轮 M3508 (C620)
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(friction_motor_group); // 摩擦轮广播组（一条总线一组）
DRVLIB_MOTOR_INSTANCE_DEF(friction_motor_ctrl);          // 摩擦轮速度环（三级 PID 核的中间环）
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(trigger_motor);     // 拨弹 M3508 (C620)
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(trigger_motor_group);  // 拨弹广播组（CAN_1，独立于摩擦轮组）
DRVLIB_MOTOR_INSTANCE_DEF(trigger_motor_ctrl);           // 拨弹 位置环|速度环（三级 PID 核的外+中环）
// 通信
static cmd2shoot_data_t shoot_cmd2shoot_data; // cmd-shoot
static shoot2cmd_data_t shoot_shoot2cmd_data; // shoot-cmd
// 数据
static float friction_motor_setref = 0; // 摩擦轮目标转速 (rad/s)：本阶段无来源，恒 0
static float trigger_omega_ref = 0;     // 拨弹位置环速度前馈 (rad/s)：轨迹已摘除，恒 0（指针仍挂在控制核配置里）
static float trigger_friction_ff = 0;   // 拨盘摩擦前馈 (A)：speed_feedforward 指针指向它，本阶段恒 0

/*============================================
 *        拨弹状态机（lib_fsm_table）
 *   状态：关 / 单发 / 三发 / 连发 / 停止
 *   事件：ch7 上升沿 / ch7 下降沿 / 拨盘走过本状态要求的一段距离
 *   设计要点：
 *   - 发射模式（ch9）只在「关 + ch7 上升沿」这一拍读取，之后锁在所进的状态里；中途改
 *     ch9（甚至改模式）都不影响正在走的这一发。
 *   - 单发/三发在走完当前这一发之前收到 ch7 下降沿 → 不打断，本发走完时再判：
 *     那时 ch7 若仍断开就回「关」（三发余下的发数取消），仍闭合则进「停止」。
 *     连发/停止 收到 ch7 下降沿 → 立即回「关」。
 *   - "走过一定距离"= 自本发起累计的机构行程达 TRIGGER_STEP_RAD（见 TriggerFsmUpdate）。
 *============================================*/
typedef enum : uint8_t
{
    trigger_state_off_e = 0, // 关：ch7 断开，不动作
    trigger_state_single_e,  // 单发：走完 1 发
    trigger_state_triple_e,  // 三发：走完 3 发（逐发起停，便于中途取消余下发数）
    trigger_state_auto_e,    // 连发：一发接一发不停
    trigger_state_stop_e,    // 停止：本状态要求的行程已走完，等 ch7 断开
    trigger_state_count_e,   // 状态值域上界（= 进入动作表项数）
} trigger_state_e;

typedef enum : uint8_t
{
    trigger_ev_fire_on_e = 0, // ch7 0→1
    trigger_ev_fire_off_e,    // ch7 1→0
    trigger_ev_step_done_e,   // 拨盘走过本状态要求的一段距离
} trigger_event_e;

/* 状态机上下文：模式与位置在每拍派发前由 TriggerFsmUpdate 刷新 */
typedef struct
{
    fire_mode_e fire_mode; // 本拍 cmd 下发的发射模式（off 同时表示"ch7 断开"）
    uint8_t fire_last;     // 上一拍 ch7 电平（上升/下降沿检测）
    uint8_t pos_valid;     // last_pos 是否已初始化（首帧不产生增量）
    uint8_t shots_left;    // 本状态还剩几发（单发=1、三发=3 逐发递减；连发/停止不用）
    float last_pos;        // 上一拍拨盘位置 (rad，app 正向，单圈)
    float step_travel;     // 自本发起累计的机构行程 (rad)，满 TRIGGER_STEP_RAD 即一步走完
} TriggerFsmCtx_s;

static TriggerFsmCtx_s trigger_fsm_ctx;
static LibFsmTableInstance_s trigger_fsm;

/*---------------- 状态动作（进入/退出） ----------------*/
/* 进入任何状态都把"本发的行程"清零：让新状态的 E_STEP_DONE 从机构当前位置重新起算 */
static void TriggerFsmEnterState(void *ctx)
{
    ((TriggerFsmCtx_s *)ctx)->step_travel = 0.0f;
}
static void TriggerFsmEnterSingle(void *ctx)
{
    TriggerFsmCtx_s *c = ctx;
    TriggerFsmEnterState(ctx);
    c->shots_left = 1;
}
static void TriggerFsmEnterTriple(void *ctx)
{
    TriggerFsmCtx_s *c = ctx;
    TriggerFsmEnterState(ctx);
    c->shots_left = 3;
}
static void TriggerFsmEnterAuto(void *ctx)
{
    TriggerFsmCtx_s *c = ctx;
    TriggerFsmEnterState(ctx);
    c->shots_left = 0; // 连发不计数，靠 ch7 断开停
}

/*---------------- 转移动作 ----------------*/
/* 三发/连发：本发走完、ch7 仍闭合 → 原地起下一发（内部转移，状态不退出也不重新进入） */
static void TriggerFsmNextShot(void *ctx, const void *event_data)
{
    (void)event_data;
    TriggerFsmCtx_s *c = ctx;
    if (c->shots_left > 0)
    {
        c->shots_left--;
    }
    c->step_travel = 0.0f;
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
static bool TriggerFsmGuardHasNextShot(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((TriggerFsmCtx_s *)ctx)->shots_left > 1;
}

/*---------------- 转移表（表顺序 = 优先级） ----------------*/
static const LibFsmTableTransition_s trigger_fsm_table[] = {
    /* 1~3. 关 + ch7 上升沿 → 按 ch9 解出的模式进对应状态（三条同 from/event，守卫依次求值） */
    {trigger_state_off_e, trigger_ev_fire_on_e, trigger_state_single_e, TriggerFsmGuardSingle, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_off_e, trigger_ev_fire_on_e, trigger_state_triple_e, TriggerFsmGuardTriple, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_off_e, trigger_ev_fire_on_e, trigger_state_auto_e, TriggerFsmGuardAuto, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    /* 4. 走过一定距离 → 单发/三发进停止或回关；三发/连发起下一发 */
    {trigger_state_single_e, trigger_ev_step_done_e, trigger_state_off_e, TriggerFsmGuardFireOff, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_single_e, trigger_ev_step_done_e, trigger_state_stop_e, NULL, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_triple_e, trigger_ev_step_done_e, trigger_state_off_e, TriggerFsmGuardFireOff, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_triple_e, trigger_ev_step_done_e, trigger_state_triple_e, TriggerFsmGuardHasNextShot, TriggerFsmNextShot, LIB_FSM_TABLE_TRANS_INTERNAL},
    {trigger_state_triple_e, trigger_ev_step_done_e, trigger_state_stop_e, NULL, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {trigger_state_auto_e, trigger_ev_step_done_e, trigger_state_auto_e, NULL, TriggerFsmNextShot, LIB_FSM_TABLE_TRANS_INTERNAL},
    /* 5. ch7 下降沿 → 关。单发/三发**不在这里回关**：本发要走完，"走完"那条规则上再判
     *    ch7 电平（见上面的 TriggerFsmGuardFireOff 规则）—— 这就是"走完当前这一发，
     *    余下取消"的落地方式。 */
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
 *           拨盘行程 → "走过一定距离"。
 * 注意：本阶段拨盘电机固定给 0（见 AppShootRun），机构不会动，故 E_STEP_DONE 实际不会
 *       产生 —— 单发/三发走不完就停在原状态等（这正是"走完当前这一发"的定义），
 *       接回电机通路后自动恢复。 */
static void TriggerFsmUpdate(float pos)
{
    TriggerFsmCtx_s *c = &trigger_fsm_ctx;
    uint8_t fire_now;

    /* 1. 刷新本拍输入：模式 + 行程。位置反馈是单圈值，逐拍增量做半圈回绕处理，
     *    跨界（0↔2π）不会被算成一段行程。 */
    c->fire_mode = shoot_cmd2shoot_data.fire_mode;
    if (c->pos_valid)
    {
        float d = pos - c->last_pos;
        if (d > TRIGGER_POS_WRAP_HALF)
        {
            d -= TRIGGER_POS_WRAP_FULL;
        }
        else if (d < -TRIGGER_POS_WRAP_HALF)
        {
            d += TRIGGER_POS_WRAP_FULL;
        }
        c->step_travel += fabsf(d);
    }
    c->last_pos = pos;
    c->pos_valid = 1;

    /* 2. ch7 边沿（fire_mode == off 即 ch7 断开，两者由 cmd 编在同一字段里，无需另传电平） */
    fire_now = (c->fire_mode != fire_mode_off_e) ? 1 : 0;
    if (fire_now != c->fire_last)
    {
        LibFsmTableDispatch(&trigger_fsm, (fire_now != 0) ? trigger_ev_fire_on_e : trigger_ev_fire_off_e, NULL);
        c->fire_last = fire_now;
    }

    /* 3. "走过一定距离"：进新状态/起下一发时行程已清零，故一拍至多触发一次。
     *    停在关/停止时机构仍可能滑行（无人在意行程），此时查表无人接手 → 把行程清掉，
     *    免得每拍都拿一个过期的 step_done 去空转查表。 */
    if (c->step_travel >= TRIGGER_STEP_RAD && !LibFsmTableDispatch(&trigger_fsm, trigger_ev_step_done_e, NULL))
    {
        c->step_travel = 0.0f;
    }
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

    /* 控制核：位置环(外)|速度环(内)（ref = rad 目标角 → 位置 PID → 速度 PID → 电流）。
     * 本阶段不接在控制回路上（电机固定给 0），配置留在这里备用 —— 接回时由状态机给出
     * 目标角/速度、恢复 AppShootRun 里那一调用即可。 */
    DrvlibMotor_Config_s trigger_motor_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED, // 位置(外)→速度(内)
        .position_mode = DRVLIB_MOTOR_POS_CONTINUOUS,                   // 连续模式，不引入限幅
        .angle_limit_max = 0,
        .angle_limit_min = 0,
        .position_span = 6.2831853f, // 单圈 2π（M3508 编码器跨度）
        .position_offset = 0,
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE, // 速度/电流低通
        /* 速度低通 5 ms（由 20 ms 下调）：极限环主频 6.5 Hz（41 rad/s）处，20 ms 一阶
         * 低通要吃掉 atan(41×0.02)=39° 相位，是环路里最大的一处滞后。拨盘编码器反馈本身
         * 很干净（静止段 meas 恒为 0.000，运行纹波仅 0.16 rad/s），没必要滤那么狠。
         * 波特图上看，把 4 倍相位（≈28°）还给环路，是提高相位裕度最直接的一手。 */
        .lpf_rc = 0.005f,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_EXTERNAL, // 位置环前馈 = 轨迹 ω_ref
        .speed_feedforward_src = DRVLIB_MOTOR_FF_EXTERNAL, // 速度环前馈 = 拨盘摩擦补偿
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .angle_feedforward_ptr = &trigger_omega_ref,   // 前馈指针（轨迹速度）
        .speed_feedforward_ptr = &trigger_friction_ff, // 前馈指针（摩擦力矩，力矩域）
        .current_feedforward_ptr = NULL,
        .pid =
            {
                // 位置环：只给 kp（比例→速度设定），不加 ki（保持力矩靠内层速度环积分）
                [DRVLIB_MOTOR_STAGE_ANGLE] =
                    {
                        .kp = 5.0,                 // 比例系数（占位，实机整定）
                        .ki = 0,                   // 不加积分
                        .kd = 0,                   // 内层速度反馈已提供阻尼，暂不加微分
                        .out_max = TRIGGER_V_MAX,  // 输出限幅 = 速度上限（ω_ref 也受此约束）
                        .out_min = -TRIGGER_V_MAX, // 输出下限
                        .config_mask = PID_ENABLE_OUTPUT_LIMIT | PID_ENABLE_TRAPEZOID_INTEGRAL,
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

    // 拨弹状态机：初始「关」（ch7 断开）。Start 会跑一次「关」的进入动作（行程清零）
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
     *       起步就是一脚踢。两种状态下电机都在收帧，C620 不掉线。 */
    DrvsDJIMotorBroadcastData_s fm = DrvsDJIMotorBroadcastGetData(&friction_motor);
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
        float friction_out = DrvlibMotorSetRef(&friction_motor_ctrl, friction_motor_setref, &friction_fb, dt * 0.001f);
        DrvsDJIMotorBroadcastSetRef(&friction_motor, friction_out);
    }
    else // 之后考虑刹车
    {
        friction_motor_setref = 0.0f;
        DrvlibMotorReset(&friction_motor_ctrl); // 清 PID（含积分）与 LPF 状态，不动 enable 标志
        DrvsDJIMotorBroadcastSetRef(&friction_motor, 0.0f);
    }
    DrvsDJIMotorBroadcastGroupSend(&friction_motor_group);

    // ---- 拨盘 ----
    /* 机械方向取反（2026-10-10 看实机定）：拨盘"正转"应为另一个物理旋向。反馈与输出同时
     * 取负号 = 重定义正方向，闭环不变号（仍负反馈）、物理旋向翻转。只反输出会变成正反馈。 */
    DrvsDJIMotorBroadcastData_s t = DrvsDJIMotorBroadcastGetData(&trigger_motor);
    DrvlibMotorFeedback_s trigger_fb = {
        .position = -t.position,
        .speed = -t.speed,
        .current = t.current,
        .timestamp_us = t.timestamp_us,
    };
    /* 状态机（本阶段拨盘的唯一逻辑）：ch7/ch9 电平 → 事件 → 状态转移；"走过一定距离"
     * 由拨盘原始位置反馈的行程判定。电机不给输出（固定 0）：位置/速度环与轨迹规划都
     * 未接入，trigger_omega_ref / trigger_friction_ff 保持 0，控制核配置留在 AppShootInit。 */
    TriggerFsmUpdate(trigger_fb.position);
    DrvsDJIMotorBroadcastSetRef(&trigger_motor, 0.0f);
    DrvsDJIMotorBroadcastGroupSend(&trigger_motor_group);

    /* 调试：拨弹状态机 + 拨盘速度环观测通道（本块自己发帧，同时 app_gimbal 的 IMU 那路
     * VOFA 已关，见那里的 .vofa_enable）。
     * 本阶段拨盘电机通路停用（电机固定 0、lib_traj 已摘除）：控制核不再被调用，CH1~CH8
     * 恒 0、CH10 恒 0 —— 通道定义保留，接回电机通路即可用。全部统一到 "app 正向约定"：
     *   CH1   本发起累计行程 (rad)：到 TRIGGER_STEP_RAD 即"走过一定距离"（状态机的步进判据）
     *   CH2-3 实际/误差：看跟随快慢(带宽)、超调量、稳态差
     *   CH4-7 总输出/P/I/D：看各项谁在主导（kd=0 时 CH7 应恒为 0；CH6 稳态应停在维持电流上）
     *   CH8   实测电流 (A)：已取反到 app 正向约定（电机反馈电流是电机极性，方向取反后
     *         与 CH4 反号），故与 CH4 应基本重合；对不上说明电流量纲/方向有问题
     *   CH9   本拍 dt (ms)：确认控制周期；有跳变说明任务被拖或 CAN 掉帧
     *   CH10  摩擦前馈 (A)：本阶段恒 0（计算已摘除，参数见 TRIGGER_FRICTION_FF* 宏）
     *   CH11  状态机当前状态 (trigger_state_e)：0 关 / 1 单发 / 2 三发 / 3 连发 / 4 停止
     *   CH12  原始反馈转速 (rad/s)，未过低通：低速粘滑判据用的就是它（静摩擦前馈恢复后） */
    const PIDInstance *trigger_spd_pid = &trigger_motor_ctrl.pid[DRVLIB_MOTOR_STAGE_SPEED];
    VofaSetChannel(1, trigger_fsm_ctx.step_travel);              // 本发起累计行程 (rad)
    VofaSetChannel(2, trigger_spd_pid->measure);                 // 实际转速 (rad/s)，PID 真正用的反馈
    VofaSetChannel(3, trigger_spd_pid->error);                   // 误差 (rad/s) = 目标 - 实际
    VofaSetChannel(4, trigger_spd_pid->output);                  // PID 总输出 = 电流指令 (A)，含前馈
    VofaSetChannel(5, trigger_spd_pid->p_out);                   // 比例项 (A)
    VofaSetChannel(6, trigger_spd_pid->i_out);                   // 积分项 (A)
    VofaSetChannel(7, trigger_spd_pid->d_out);                   // 微分项 (A)
    VofaSetChannel(8, -trigger_motor_ctrl.data.current);         // 实测电流 (A)，取反到 app 正向
    VofaSetChannel(9, trigger_spd_pid->dt * 1000.0f);            // 本拍 dt (ms)
    VofaSetChannel(10, trigger_friction_ff);                     // 摩擦前馈总额 (A) = F_c + b·|ω_ref|
    VofaSetChannel(11, (float)LibFsmTableCurrent(&trigger_fsm)); // 状态机当前状态
    VofaSetChannel(12, trigger_fb.speed);                        // 原始反馈转速 (rad/s)
    VofaSend();

    // 3. 发送
    xQueueOverwrite(shoot2cmd_queue_handle, &shoot_shoot2cmd_data); // 通过队列
}
