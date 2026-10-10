#ifndef __APP_CMD_SHOOT_H
#define __APP_CMD_SHOOT_H

#include <stdint.h>

#include "app.h"          /* robot_mode（cmd2shoot_data_t 的状态码字段） */
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

/*============================================
 *              结构体
 *   cmd → shoot / shoot → cmd 均为板上队列，不上线，无需字节对齐。
 *   本版为对照射机构（射频 + 热量）+ 裁判系统校准重写的接口；
 *   旧的调试字段（fire_or_not / bullet_speed / debug_knob / temp_unused）已删。
 *   两侧调用点：app_cmd.c 已按 fire_mode 下发；app_shoot.c 已接成拨弹状态机。
 *   尚未接线：fire_rate 与两个裁判校准量（cmd 侧暂无来源）、shoot2cmd 的两个回传量。
 *============================================*/
typedef struct
{
    robot_mode mode;       // 机器人模式（stop/normal/gyro/hole），shoot 侧尚无使用点
    fire_mode_e fire_mode; // 发射模式
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
} shoot2cmd_data_t;

#endif // !__APP_CMD_SHOOT_H
