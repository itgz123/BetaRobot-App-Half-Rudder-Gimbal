#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/*============================================
 *              任务创建函数
 *============================================*/
void function_in_main_c(void);

#define CMD_DEBUG_STAGE_gimbal 0b00000001
#define CMD_DEBUG_STAGE_shoot 0b00000010
#define CMD_DEBUG_STAGE_chassis 0b00000100
#define CMD_DEBUG_STAGE_referee 0b00001000
#define CMD_DEBUG_STAGE CMD_DEBUG_STAGE_gimbal | CMD_DEBUG_STAGE_shoot | CMD_DEBUG_STAGE_chassis //

/*============================================
 *              机器人状态机模式
 *   （全 app 通用：cmd / gimbal / 底盘云台链路都以此为状态码）
 *============================================*/
typedef enum : uint8_t
{
    robot_mode_stop = 0,
    robot_mode_normal = 1,
    robot_mode_gyro = 2,
    robot_mode_hole = 3,
    /* 自由驱动：只发给底盘（云台自己的状态机不认识这个值，cmd 绝不会把它写进
     * cmd2gimbal_data）。底盘照常 RUN，但不做 θ 跟随、w=0，vx/vy 按车身系直通。
     * 对端定义必须一致：app/half_rudder_chassis/app/app.h */
    robot_mode_free = 4,
} robot_mode; // 状态机 参考`app/half_rudder_gimbal/README.md`

/*============================================
 *              链路接口（按需 include）
 *   本文件只放全 app 通用内容；各条链路的队列句柄与数据结构
 *   分列于同目录 app_cmd_<对端>.h，用到的 .c 自己 include：
 *     app_cmd_gimbal.h   cmd  ↔ gimbal（队列，板上）
 *     app_cmd_shoot.h    cmd  ↔ shoot （队列，板上）
 *     app_cmd_visual.h   cmd  ↔ 视觉电脑（线协议）
 *     app_cmd_chassis.h  云台 ↔ 底盘（CAN 线协议，两端定义须对齐）
 *============================================*/

#endif // !__APP_H
