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
 *   失能：三轴输出 0；
 *   立起中：下 pitch 用速度环驶向立起端，上 pitch 跟随 cmd 设定值；
 *   直立：下 pitch = 重力前馈 + 固定偏置 + 上 pitch 单向耦合；
 *   倒下中：等 yaw 相对底盘对正后，下 pitch 用速度环驶向倒下端、上 pitch 驶向 0
 *           （未对正时下 pitch 速度给定 0，只做阻尼）；
 *   倒下：下 pitch 输出 0（关闭重力前馈），上 pitch 保持在 0；
 *         yaw 在除失能外的所有状态继续闭环。 */
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
 * pitch 与 yaw 都是**世界系**量，cmd 不做任何坐标系换算：
 *   pitch_world / pitch_vel —— 上 pitch 指向/角速度（rad, rad/s），ZYX 符号（正=低头），
 *                             即视觉线协议 pitch 的 rad 版。已含基座倾角与安装偏置。
 *   yaw_position / yaw_vel  —— IMU 世界系航向（rad, rad/s），与视觉世界系 yaw 同系。
 * yaw_motor_* 是另一套量（云台相对底盘的关节角），别与上面混用：
 *   无减速箱，位置为单圈绝对角、已 wrap 到 ±π（不做软件多圈累加）。
 *   cmd 用它在本地算实测 θ（扣机械零位偏置）并旋转摇杆向量；
 *   底盘不接收实测角，而是自己从共享 CAN1 读同一颗 RS05。
 *   帧里的 gimbal2chassis_data.yaw 是**目标偏角** θ_ref
 *   （normal/hole = 0，gyro 斜坡），见 app_cmd.c 的
 *   chassis_yaw_target_from_mode 与底盘 ChassisWCommand。 */
typedef struct
{
    gimbal_state_e state;      // 云台实际姿态（失能/立起中/直立/倒下中/倒下）
    float pitch_world;         // 上pitch世界系指向 (rad)，正=低头；= 视觉线协议 pitch
    float pitch_vel;           // 上pitch世界系角速度 (rad/s)，与 pitch_world 同号
    float yaw_position;        // yaw轴当前世界系航向 (rad)，来自 IMU
    float yaw_vel;             // yaw轴当前世界系航向角速度 (rad/s)，来自 IMU+陀螺仪投影
    float pitch_down_position; // 下pitch轴当前反馈位置 (rad)
    float yaw_motor_position;  // yaw电机编码器关节角 (rad)，云台相对底盘（预留，cmd 侧暂未使用）
    float yaw_motor_vel;       // yaw电机编码器角速度 (rad/s)，云台相对底盘（预留，cmd 侧暂未使用）
} gimbal2cmd_data_t;
typedef struct
{
    robot_mode mode; // 状态机
    float pitch_x;   // 上pitch世界系设定指向 (rad)，正=低头；gimbal 侧换算关节角并限位
    float pitch_v;   // 上pitch世界系设定角速度 (rad/s)，正=低头
    float pitch_a;   // 上pitch世界系设定角加速度 (rad/s²)
    float yaw_x;     // yaw轴设定位置
    float yaw_v;     // yaw轴设定速度
    float yaw_a;     // yaw轴设定加速度
    /* 底盘实测速度 (m/s，车体系)：只在本板内部经队列转给 app_gimbal，
     * 由它差分得到底盘加速度做加速度计门控。不在 CAN 线上、不改底盘协议。 */
    float chassis_vx;
    float chassis_vy;
} cmd2gimbal_data_t;

#endif // !__APP_CMD_GIMBAL_H
