#ifndef __APP_CMD_SHOOT_H
#define __APP_CMD_SHOOT_H

#include <stdint.h>

#include "bsp_freertos.h" /* QueueHandle_t */

/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t cmd2shoot_queue_handle;
extern QueueHandle_t shoot2cmd_queue_handle;

/*============================================
 *              枚举
 *============================================*/
/* 发射模式（遥控档位 → cmd → shoot）：关 / 单发 / 三连发 / 全自动 */
typedef enum : uint8_t
{
    fire_mode_off_e = 0,    // 关
    fire_mode_single_e = 1, // 单发
    fire_mode_triple_e = 2, // 三连发
    fire_mode_auto_e = 3,   // 全自动
} fire_mode_e;

/* 摩擦轮工作模式（cmd → shoot 的三段指令）。
 * 0 必须是 disable：cmd_cmd2shoot_data 是零初始化，首帧下发前保持"失能"最安全。 */
typedef enum : uint8_t
{
    shoot_friction_disable_e = 0,  // 失能：不跑控制核，直接 0 电流（拨盘同样只发 0 电流）
    shoot_friction_spin_e = 1,     // 使能：速度环满速（FRICTION_SPEED_FULL）
    shoot_friction_slowdown_e = 2, // 缓慢减速：参考按固定减速度斜坡降到 0，速度环跟随
} shoot_friction_mode_e;

/* 摩擦轮转速是否已低于「过洞允许倒下」阈值（shoot → cmd）。
 * cmd 切到 hole 档后先让摩擦轮缓慢减速，等本值变成 low 才真正把 hole 下发给云台；
 * 阈值由 shoot 侧（app_shoot_def.h 的 FRICTION_SPEED_HOLE_THRESHOLD）判定。 */
typedef enum : uint8_t
{
    shoot_friction_speed_high_e = 0, // 仍高于阈值（或尚未收到反馈，保守不放行）
    shoot_friction_speed_low_e = 1,  // 已低于阈值，允许云台倒下
} shoot_friction_speed_e;

/*============================================
 *              结构体
 *   cmd → shoot / shoot → cmd 均为板上队列，不上线，无需字节对齐。
 *   本版为对照射机构（射频 + 热量）+ 裁判系统校准重写的接口；
 *   旧的调试字段（fire_or_not / bullet_speed / debug_knob / temp_unused）已删。
 *   两侧调用点：app_cmd.c 已按 fire_mode 下发；app_shoot.c 已接成拨弹状态机。
 *   friction_mode 由 cmd 侧算（见 app_cmd.c 的 send_shoot）：总开关(ch4) 打开 且 云台状态为
 *   直立时才满速转；过洞档给"缓慢减速"；失能/立起中/倒下等给"失能"（0 电流，拨盘同步 0 电流）。
 *   ch7 电平不再单独下传：已全部编进 fire_mode —— off = ch7 断开（或 ch4 关/掉线），非 off
 *   = ch7 闭合（且 ch9 已解出 单发/三连发/全自动）。shoot 侧判"开火总闸闭合"用
 *   fire_mode != fire_mode_off_e 即可。
 *   尚未接线：fire_rate 与两个裁判校准量（cmd 侧暂无来源）、shoot2cmd 的
 *   remaining_heat / bullets_fired（friction_speed 已接线，用于过洞放行）。
 *============================================*/
typedef struct
{
    shoot_friction_mode_e friction_mode; // 摩擦轮模式：失能(0A) / 满速 / 缓慢减速到 0
    fire_mode_e fire_mode;               // 发射模式（ch7 闭合时由 ch9 三段解出；ch7 断开 = off）
    float fire_rate;       // 射频 (发/s)

    /* 裁判系统校准量：cmd 收到裁判消息后原样下传，shoot 侧拿去校正本地弹道/热量模型 */
    float referee_heat;                 // 裁判系统枪口热量 (热量单位)
    uint64_t referee_heat_timestamp_us; // 该热量消息的到达时间戳 (us，DWT 时间轴)
    float referee_bullet_speed;         // 裁判系统测得弹速 (m/s)
} cmd2shoot_data_t;
typedef struct
{
    float remaining_heat;   // 剩余枪口热量 (热量单位)
    uint32_t bullets_fired; // 累计发弹数 (发)
    /* 摩擦轮转速是否已低于过洞阈值：cmd 用它决定何时把 hole 真正下发给云台。
     * 默认 0 = high：首帧反馈到达前不放行倒下。 */
    shoot_friction_speed_e friction_speed_low;
} shoot2cmd_data_t;

#endif // !__APP_CMD_SHOOT_H
