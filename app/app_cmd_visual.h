#ifndef __APP_CMD_VISUAL_H
#define __APP_CMD_VISUAL_H

#include <stdint.h>

/*============================================
 *   视觉电脑通信：业务数据结构
 *   线协议后端在 app_comm_proto_visual/app_proto_visual.h（PROTO_VISUAL，仅编解码）。
 *   本文件只管"帧里装什么"，与线协议的字节布局强绑定，改字段须与视觉端同步。
 *============================================*/

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
 *            视觉通信协议数据结构
 *   （与线协议字节对齐：payload 即帧去 CRC16 的完整字节，
 *    packed 无 padding，memcpy 到结构体即得业务字段）
 *============================================*/
#pragma pack(push, 1)

/* 接收帧（视觉→板）：payload 48B，线协议帧 = 结构体 + CRC16(2B) = 50B。
 * 对应视觉端结构体 StandardSendRobotCmdData（字段顺序/类型逐一对齐）。
 * 字段名使用机器人内部统一语义：线协议裸字段 pitch/target_pitch 解释为**世界系**
 * 上 pitch 指向（视觉做世界系解算需要基座姿态）；板侧原样下发给 gimbal，
 * 由 gimbal 换算成"相对云台基座"的关节角并在基座相对坐标里限幅。
 * v_pitch/a_pitch 实测是"向上为正"（与位置字段镜像），板侧取负换成世界系 ZYX 符号。 */
typedef struct
{
    vision_cmd_e cmd_ID;        /* 帧定界命令字（= VISUAL_CMD_RX） */
    uint32_t time_stamp;        /* 时间戳 ms */
    vision_appear_state appear; /* 目标是否出现 */
    uint8_t shoot_rate;         /* 发射速率档（视觉端默认 3） */
    /* 单位约定：yaw/pitch_base_relative/target_yaw/target_pitch_base_relative 为 deg；
     * v_yaw/v_pitch_base_relative 为 rad/s；a_yaw/a_pitch_base_relative 为 rad/s² */
    /* 符号约定：pitch 位置字段在线上是“向上为负”（视觉约定），与 ZYX 欧拉 pitch
     * 同号（正=低头），板侧/云台内部统一用这个世界系符号。
     * 注意：实测视觉发送的 v_pitch/a_pitch 与位置字段是**镜像**（v/a 向上为正），
     * 故板侧对 v/a 取负后再用；速度已是世界系量，gimbal 再加基座角速度换算到关节。 */
    float yaw;                        /* 视觉给出的世界系 yaw（视觉端 yaw，deg） */
    float pitch_base_relative;        /* 视觉给出的**世界系**上 pitch 指向（视觉端 pitch，deg；向上为负）。
                                       * 字段名沿用旧名，线协议字节不变 */
    float target_yaw;                 /* 视觉预测的目标世界系 yaw（视觉端 target_yaw，deg） */
    float target_pitch_base_relative; /* 视觉预测的目标世界系 pitch（视觉端 target_pitch，deg；向上为负） */
    float enable_yaw_diff;            /* 是否发弹。0:不发弹；1:发弹（连发） */
    float enable_pitch_diff;          /* 弹频。1:每秒2发；2:每秒4发；3:每秒6发 */
    float v_yaw;                      /* yaw 角速度前馈（视觉端 v_yaw，rad/s） */
    float v_pitch_base_relative;      /* 世界系 pitch 角速度前馈（视觉端 v_pitch，rad/s；向上为正） */
    float a_yaw;                      /* yaw 角加速度前馈（视觉端 a_yaw，rad/s²） */
    float a_pitch_base_relative;      /* 世界系 pitch 角加速度前馈（视觉端 a_pitch，rad/s²；向上为正） */
    enemy_color_e detect_color;       /* 检测到的敌方颜色（视觉端 detect_color：0 red / 1 blue） */
} vision_recv_t;

/* 发送帧（板→视觉）：payload 55B，线协议帧 = 结构体 + CRC16(2B) = 57B。
 * 对应视觉端结构体 ReceiveRobotData（字段顺序/类型逐一对齐）。
 * 字段名使用机器人内部统一语义：线协议裸字段 pitch 解释为**世界系**上 pitch 指向
 * （已含云台基座姿态）；pitch_vel 也是世界系角速度，按视觉约定"向上为正"发送。
 * aim_color 对应视觉端 enemy_color。 */
typedef struct
{
    vision_cmd_e cmd_ID; /* 帧定界命令字（= VISUAL_CMD_TX） */
    uint32_t time_stamp; /* 时间戳 ms */
    vision_mode_e mode;  /* 工作模式（视觉端 mode：0 IDLE / 1 AUTO_AIM / 2 SMALL_BUFF / 3 BIG_BUFF） */
    /* 单位约定：yaw/pitch_base_relative/pitch_down/roll 为 deg；
     * yaw_vel/pitch_base_relative_vel/roll_vel 为 rad/s；
     * 符号约定：pitch_base_relative 用视觉约定“向上为负”（与 ZYX 欧拉 pitch 同号）；
     * v_x/v_y/v_z、bullet_speed 为 m/s */
    float yaw;                     /* 发给视觉的世界系 yaw（视觉端 yaw，deg） */
    float pitch_base_relative;     /* 发给视觉的**世界系**上 pitch 指向（视觉端 pitch，deg；向上为负）。
                                    * 字段名沿用旧名，线协议字节不变 */
    float pitch_down;              /* 下 pitch 机构位姿角（视觉端 pitch_down，deg） */
    float roll;                    /* 车体/IMU roll（视觉端 roll，deg） */
    float yaw_vel;                 /* yaw 角速度（视觉端 yaw_vel，rad/s） */
    float pitch_base_relative_vel; /* 世界系 pitch 角速度（视觉端 pitch_vel，rad/s；向上为正） */
    float roll_vel;                /* 车体/IMU roll 角速度（视觉端 roll_vel，rad/s） */
    float v_x;                     /* 车体 x 速度（视觉端 v_x，m/s） */
    float v_y;                     /* 车体 y 速度（视觉端 v_y，m/s） */
    float v_z;                     /* 车体 z 速度（视觉端 v_z，m/s） */
    float bullet_speed;            /* 弹速（视觉端 bullet_speed，m/s） */
    uint32_t bullet_count;         /* 弹量（视觉端 bullet_count） */
    enemy_color_e aim_color;       /* 瞄准敌方颜色（视觉端 enemy_color：0 red / 1 blue） */
} vision_send_t;

#pragma pack(pop)

#endif // !__APP_CMD_VISUAL_H
