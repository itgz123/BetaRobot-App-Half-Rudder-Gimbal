#ifndef __APP_CMD_CHASSIS_H
#define __APP_CMD_CHASSIS_H

#include "app.h" /* robot_mode */

/*============================================
 *            底盘-云台 CAN 通信数据结构
 *   （CAN1 / idseq / PROTO_CUSTOM：payload 即业务结构体，
 *   线上帧 = [0xA5][seq][payload N][CRC8][0x5A]，N = sizeof(结构体)）
 *============================================*/
#pragma pack(push, 1)
/* 云台→底盘（18B）与底盘→云台（12B）各一结构体；两端定义须字节对齐
 * （COMM_DEF 内 _Static_assert 校验 sizeof == 约定线长）。
 * 本板（云台）在 app_cmd.c 收发包，对端定义见 app/half_rudder_chassis/app/app.h。 */
// 方向定义：vx>0 前；vy>0 左；w>0 俯视逆时针（底盘侧实车已核对：w>0 确实逆时针）
typedef struct
{
    robot_mode enabled;
    // 设定速度
    float vx;
    float vy;
    /* 底盘角速度前馈 (rad/s，逆时针为正)：云台 yaw 电机速度的指令前馈，
     * 方便底盘直接合成控制量。normal/hole 是云台 yaw 指令角速度；gyro 是定值
     * 自转角速度；调试直通时是 yaw 摇杆直控。 */
    float w;
    /* 云台与底盘的**目标相对角** θ_ref (rad)，逆时针为正：θ = ψ_云台 − ψ_底盘。
     * normal/hole 固定 0；gyro 按 −自转角速度斜坡变化；调试直通时填当前实测角。
     * 底盘不用它当实测角：底盘自己从共享 CAN1 直接读 RS05 反馈、扣同一个
     * chassis_gimbal_offset 得到实测 θ，再用 wrap(θ − θ_ref) 做跟随。 */
    float yaw;
    uint8_t power_boost_enable; // 1 = 在基础功率上限上加额外功率（遥控 ch5，由云台转发给底盘）
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
