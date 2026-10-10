#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/*============================================
 *              任务创建函数
 *============================================*/
void function_in_main_c(void);

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
