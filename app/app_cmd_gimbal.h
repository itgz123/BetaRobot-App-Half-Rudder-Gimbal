#ifndef __APP_CMD_GIMBAL_H
#define __APP_CMD_GIMBAL_H

#include "app.h"          /* robot_mode（cmd2gimbal_data_t 的状态码字段） */
#include "bsp_freertos.h" /* QueueHandle_t */

/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t gimbal2cmd_queue_handle;
extern QueueHandle_t cmd2gimbal_queue_handle;

/*============================================
 *              枚举
 *============================================*/
/* 云台状态机所处状态（gimbal → cmd 反馈）：报的是**机构实际姿态**，不是 cmd 下发的档位。
 * 驱动量有两个 —— cmd 的 robot_mode 定目标姿态（stop=失能、normal/gyro=直立、hole=倒下），
 * 下 pitch 位置定实际姿态；两者不一致时就落在两个过渡态上。
 * 行为差异（见 app_gimbal.c 的云台状态机）：
 *   失能：三轴输出 0；立起中/直立：三轴闭环跟随 cmd 设定值；
 *   倒下中/倒下：pitch 上下两轴 0（松劲、靠机构自重下垂），yaw 继续闭环。 */
typedef enum : uint8_t
{
    gimbal_state_disable_e = 0, // 失能
    gimbal_state_rising_e = 1,  // 立起中（目标直立、机构还没抬到位）
    gimbal_state_stand_e = 2,   // 直立
    gimbal_state_falling_e = 3, // 倒下中（目标倒下、机构还没落到位）
    gimbal_state_lying_e = 4,   // 倒下
} gimbal_state_e;

/*============================================
 *              结构体
 *============================================*/
/* 云台→cmd 反馈（队列，不上线，无需字节对齐）。
 * yaw 有两套反馈，坐标系不同，别混用：
 *   yaw_position / yaw_vel —— IMU 世界系航向（rad, rad/s）。与视觉下发的世界系 yaw 同一坐标系，
 *                             cmd 的 yaw 规划器拿它当锚点、并原样回传给视觉。
 *   yaw_motor_*            —— yaw 电机编码器的关节角/角速度（rad, rad/s），即"云台相对底盘"。
 *                             预留：底盘跟随 w = kp*wrap(-云台相对底盘角) 要用这一对，
 *                             现在 cmd 侧还没切过来（见 app_cmd.c 的 chassis_w_from_mode）。 */
typedef struct
{
    gimbal_state_e state;      // 云台实际姿态（失能/立起中/直立/倒下中/倒下）
    float pitch_position;      // pitch轴当前反馈位置 (rad)
    float pitch_vel;           // pitch轴当前反馈速度 (rad/s)
    float yaw_position;        // yaw轴当前世界系航向 (rad)，来自 IMU
    float yaw_vel;             // yaw轴当前世界系航向角速度 (rad/s)，来自 IMU+陀螺仪投影
    float pitch_down_position; // 下pitch轴当前反馈位置 (rad)
    float yaw_motor_position;  // yaw电机编码器关节角 (rad)，云台相对底盘（预留，cmd 侧暂未使用）
    float yaw_motor_vel;       // yaw电机编码器角速度 (rad/s)，云台相对底盘（预留，cmd 侧暂未使用）
} gimbal2cmd_data_t;
typedef struct
{
    robot_mode mode; // 状态机
    float pitch_x;   // pitch轴设定位置
    float pitch_v;   // pitch轴设定速度
    float pitch_a;   // pitch轴设定加速度
    float yaw_x;     // yaw轴设定位置
    float yaw_v;     // yaw轴设定速度
    float yaw_a;     // yaw轴设定加速度
} cmd2gimbal_data_t;

#endif // !__APP_CMD_GIMBAL_H
