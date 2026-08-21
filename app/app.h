#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/* 任务创建函数 */
void function_in_main_c(void);

// 任务间通信
#include <stdint.h>

typedef enum : uint8_t
{
    disable = 0,
    enable = 1,
} cmd2gimbal_state; // 状态机

typedef struct
{
    uint8_t temp_unused;
} cmd2shoot_data_t;
typedef struct
{
    uint8_t temp_unused;
} shoot2cmd_data_t;
typedef struct
{
    uint8_t temp_unused;
} gimbal2cmd_data_t;
typedef struct
{
    cmd2gimbal_state state; // 状态机
} cmd2gimbal_data_t;
typedef struct
{
    uint8_t temp_unused;
} sensor2gimbal_data_t;
typedef struct
{
    uint8_t temp_unused;
} gimbal2sensor_data_t;
/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t cmd2shoot_queue_handle;
extern QueueHandle_t shoot2cmd_queue_handle;
extern QueueHandle_t gimbal2cmd_queue_handle;
extern QueueHandle_t cmd2gimbal_queue_handle;
extern QueueHandle_t sensor2gimbal_queue_handle;
extern QueueHandle_t gimbal2sensor_queue_handle;

/*============================================
 *            视觉通信协议数据结构
 *   （与线协议字节对齐：payload 即帧去 CRC16 的完整字节，
 *    packed 无 padding，memcpy 到结构体即得业务字段）
 *============================================*/
#pragma pack(push, 1)

/* 接收帧（视觉→板）：payload 48B，线协议帧 = 结构体 + CRC16(2B) = 50B。
 * 结构体字段名使用机器人内部统一语义；线协议裸字段 pitch/target_pitch 等
 * 均解释为"上 pitch 相对云台基座"（base-relative）。 */
typedef struct
{
    uint8_t cmd_ID;      /* 帧定界命令字（= VISUAL_CMD_RX） */
    uint32_t time_stamp; /* 时间戳 ms */
    uint8_t appear;      /* 目标是否出现 */
    uint8_t shoot_rate;  /* 发射速率档 */
    /* 单位约定：yaw/pitch_base_relative/target_yaw/target_pitch_base_relative 为 deg；
     * v_yaw/v_pitch_base_relative 为 rad/s；a_yaw/a_pitch_base_relative 为 rad/s² */
    float yaw;                        /* 视觉给出的世界系 yaw */
    float pitch_base_relative;        /* 视觉给出的总体 pitch：上 pitch 相对云台基座 */
    float target_yaw;                 /* 视觉预测的目标世界系 yaw */
    float target_pitch_base_relative; /* 视觉预测的目标总体 pitch：上 pitch 相对云台基座 */
    float enable_yaw_diff;            /* yaw 允许误差 */
    float enable_pitch_diff;          /* 与 target_pitch_base_relative 配套的允许误差 */
    float v_yaw;                      /* yaw 角速度前馈 */
    float v_pitch_base_relative;      /* base-relative pitch 角速度前馈 */
    float a_yaw;                      /* yaw 角加速度前馈 */
    float a_pitch_base_relative;      /* base-relative pitch 角加速度前馈 */
    uint8_t detect_color;             /* 检测到的敌方颜色 */
} vision_recv_t;

/* 发送帧（板→视觉）：payload 55B，线协议帧 = 结构体 + CRC16(2B) = 57B */
typedef struct
{
    uint8_t cmd_ID;      /* 帧定界命令字（= VISUAL_CMD_TX） */
    uint32_t time_stamp; /* 时间戳 ms */
    uint8_t mode;        /* 工作模式（对齐原项目 Work_Mode_e） */
    /* 单位约定：yaw/pitch_base_relative/pitch_down/roll 为 deg；
     * yaw_vel/pitch_base_relative_vel/roll_vel 为 rad/s；
     * v_x/v_y/v_z、bullet_speed 为 m/s */
    float yaw;                     /* 发给视觉的世界系 yaw */
    float pitch_base_relative;     /* 发给视觉的总体 pitch：上 pitch 相对云台基座 */
    float pitch_down;              /* 下 pitch 机构位姿角 */
    float roll;                    /* 车体/IMU roll */
    float yaw_vel;                 /* yaw 角速度 */
    float pitch_base_relative_vel; /* 发给视觉的 base-relative pitch 角速度 */
    float roll_vel;                /* roll 角速度 */
    float v_x;                     /* 车体 x 速度 */
    float v_y;                     /* 车体 y 速度 */
    float v_z;                     /* 车体 z 速度 */
    float bullet_speed;            /* 弹速 */
    uint32_t bullet_count;         /* 弹量 */
    uint8_t aim_color;             /* 瞄准敌方颜色 */
} vision_send_t;

#pragma pack(pop)

#endif // !__APP_H
