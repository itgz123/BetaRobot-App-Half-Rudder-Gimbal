#ifndef __APP_CMD_CHASSIS_H
#define __APP_CMD_CHASSIS_H

#include "app.h" /* robot_mode */

/*============================================
 *            底盘-云台 CAN 通信数据结构
 *   （CAN1 / idseq / PROTO_CUSTOM：payload 即业务结构体，
 *   线上帧 = [0xA5][seq][payload N][CRC8][0x5A]，N = sizeof(结构体)）
 *============================================*/
#pragma pack(push, 1)
/* 云台→底盘（14B）与底盘→云台（12B）各一结构体；两端定义须字节对齐
 * （COMM_DEF 内 _Static_assert 校验 sizeof == 约定线长）。
 * 本板（云台）在 app_cmd.c 收发包，对端定义见 app/half_rudder_chassis/app/app.h。 */
// 方向定义：vx>0 前；vy>0 左；w>0 俯视逆时针（底盘侧实车已核对：w>0 确实逆时针）
typedef struct
{
    robot_mode enabled;
    // 设定速度
    float vx;
    float vy;
    float w;
    uint8_t super_cap_enable; // 1 = 使能超电（遥控 ch5，由云台转发给底盘）
} gimbal2chassis_data_t;
typedef struct
{
    // 反馈速度
    float vx;
    float vy;
    float w;
} chassis2gimbal_data_t;
#pragma pack(pop)

#endif // !__APP_CMD_CHASSIS_H
