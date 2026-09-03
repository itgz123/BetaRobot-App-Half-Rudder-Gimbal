#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/*============================================
 *              任务创建函数
 *============================================*/
void function_in_main_c(void);

/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t cmd2shoot_queue_handle;
extern QueueHandle_t shoot2cmd_queue_handle;
extern QueueHandle_t gimbal2cmd_queue_handle;
extern QueueHandle_t cmd2gimbal_queue_handle;

/*============================================
 *              任务间通信
 *============================================*/
#include <stdint.h>

/*============================================
 *              枚举
 *============================================*/
typedef enum : uint8_t
{
    disable = 0,
    enable = 1,
} cmd2gimbal_state; // 状态机

/* 视觉帧命令字（帧首字节；随帧传递供业务识别方向，不再用于帧定界） */
typedef enum : uint8_t
{
    VISUAL_CMD_RX = 0x01, /* 接收帧（视觉→板）首字节 */
    VISUAL_CMD_TX = 0x02, /* 发送帧（板→视觉）首字节 */
} vision_cmd_e;

/* 工作模式（发给视觉，对齐原项目 Work_Mode_e） */
typedef enum : uint8_t
{
    vision_mode_idle_e = 0,       /* 空闲 */
    vision_mode_auto_aim_e = 1,   /* 自动瞄准 */
    vision_mode_small_buff_e = 2, /* 小能量机关 */
    vision_mode_big_buff_e = 3,   /* 大能量机关 */
} vision_mode_e;

/* 敌方/瞄准颜色（对齐视觉端协议：0 red / 1 blue） */
typedef enum : uint8_t
{
    enemy_red_e = 0,  /* 红方 */
    enemy_blue_e = 1, /* 蓝方 */
} enemy_color_e;

/* 视觉目标是否出现 */
typedef enum : uint8_t
{
    vision_not_appear_e = 0,
    vision_appear_e = 1,
} vision_appear_state;

/*============================================
 *              结构体
 *============================================*/
typedef struct
{
    uint8_t fire_or_not; // 临时开火标志
} cmd2shoot_data_t;
typedef struct
{
    uint8_t temp_unused;
} shoot2cmd_data_t;
typedef struct
{
    float pitch_position;      // pitch轴当前反馈位置 (rad)
    float pitch_vel;           // pitch轴当前反馈速度 (rad/s)
    float yaw_position;        // yaw轴当前反馈位置 (rad)
    float yaw_vel;             // yaw轴当前反馈速度 (rad/s)
    float pitch_down_position; // 下pitch轴当前反馈位置 (rad)
} gimbal2cmd_data_t;
typedef struct
{
    cmd2gimbal_state state; // 状态机
    float pitch_x;          // pitch轴设定位置
    float pitch_v;          // pitch轴设定速度
    float pitch_a;          // pitch轴设定加速度
    float yaw_x;            // yaw轴设定位置
    float yaw_v;            // yaw轴设定速度
    float yaw_a;            // yaw轴设定加速度
} cmd2gimbal_data_t;

/*============================================
 *            视觉通信协议数据结构
 *   （与线协议字节对齐：payload 即帧去 CRC16 的完整字节，
 *    packed 无 padding，memcpy 到结构体即得业务字段）
 *============================================*/
#pragma pack(push, 1)

/* 接收帧（视觉→板）：payload 48B，线协议帧 = 结构体 + CRC16(2B) = 50B。
 * 对应视觉端结构体 StandardSendRobotCmdData（字段顺序/类型逐一对齐）。
 * 字段名使用机器人内部统一语义：线协议裸字段 pitch/target_pitch/v_pitch/a_pitch
 * 均解释为"上 pitch 相对云台基座"（base-relative）。 */
typedef struct
{
    vision_cmd_e cmd_ID;        /* 帧定界命令字（= VISUAL_CMD_RX） */
    uint32_t time_stamp;        /* 时间戳 ms */
    vision_appear_state appear; /* 目标是否出现 */
    uint8_t shoot_rate;         /* 发射速率档（视觉端默认 3） */
    /* 单位约定：yaw/pitch_base_relative/target_yaw/target_pitch_base_relative 为 deg；
     * v_yaw/v_pitch_base_relative 为 rad/s；a_yaw/a_pitch_base_relative 为 rad/s² */
    float yaw;                        /* 视觉给出的世界系 yaw（视觉端 yaw，deg） */
    float pitch_base_relative;        /* 视觉给出的总体 pitch：上 pitch 相对云台基座（视觉端 pitch，deg） */
    float target_yaw;                 /* 视觉预测的目标世界系 yaw（视觉端 target_yaw，deg） */
    float target_pitch_base_relative; /* 视觉预测的目标总体 pitch：上 pitch 相对云台基座（视觉端 target_pitch，deg） */
    float enable_yaw_diff;            /* yaw 允许误差（当前 yaw 与 target_yaw 差绝对值 < 该值开火，deg） */
    float enable_pitch_diff;          /* 与 target_pitch 配套的允许误差（deg） */
    float v_yaw;                      /* yaw 角速度前馈（视觉端 v_yaw，rad/s） */
    float v_pitch_base_relative;      /* base-relative pitch 角速度前馈（视觉端 v_pitch，rad/s） */
    float a_yaw;                      /* yaw 角加速度前馈（视觉端 a_yaw，rad/s²） */
    float a_pitch_base_relative;      /* base-relative pitch 角加速度前馈（视觉端 a_pitch，rad/s²） */
    enemy_color_e detect_color;       /* 检测到的敌方颜色（视觉端 detect_color：0 red / 1 blue） */
} vision_recv_t;

/* 发送帧（板→视觉）：payload 55B，线协议帧 = 结构体 + CRC16(2B) = 57B。
 * 对应视觉端结构体 ReceiveRobotData（字段顺序/类型逐一对齐）。
 * 字段名使用机器人内部统一语义：线协议裸字段 pitch/pitch_vel 解释为
 * "上 pitch 相对云台基座"（base-relative）；aim_color 对应视觉端 enemy_color。 */
typedef struct
{
    vision_cmd_e cmd_ID; /* 帧定界命令字（= VISUAL_CMD_TX） */
    uint32_t time_stamp; /* 时间戳 ms */
    vision_mode_e mode;  /* 工作模式（视觉端 mode：0 IDLE / 1 AUTO_AIM / 2 SMALL_BUFF / 3 BIG_BUFF） */
    /* 单位约定：yaw/pitch_base_relative/pitch_down/roll 为 deg；
     * yaw_vel/pitch_base_relative_vel/roll_vel 为 rad/s；
     * v_x/v_y/v_z、bullet_speed 为 m/s */
    float yaw;                     /* 发给视觉的世界系 yaw（视觉端 yaw，deg） */
    float pitch_base_relative;     /* 发给视觉的总体 pitch：上 pitch 相对云台基座（视觉端 pitch，deg） */
    float pitch_down;              /* 下 pitch 机构位姿角（视觉端 pitch_down，deg） */
    float roll;                    /* 车体/IMU roll（视觉端 roll，deg） */
    float yaw_vel;                 /* yaw 角速度（视觉端 yaw_vel，rad/s） */
    float pitch_base_relative_vel; /* 发给视觉的 base-relative pitch 角速度（视觉端 pitch_vel，rad/s） */
    float roll_vel;                /* roll 角速度（视觉端 roll_vel，rad/s） */
    float v_x;                     /* 车体 x 速度（视觉端 v_x，m/s） */
    float v_y;                     /* 车体 y 速度（视觉端 v_y，m/s） */
    float v_z;                     /* 车体 z 速度（视觉端 v_z，m/s） */
    float bullet_speed;            /* 弹速（视觉端 bullet_speed，m/s） */
    uint32_t bullet_count;         /* 弹量（视觉端 bullet_count） */
    enemy_color_e aim_color;       /* 瞄准敌方颜色（视觉端 enemy_color：0 red / 1 blue） */
} vision_send_t;

#pragma pack(pop)

#endif // !__APP_H
