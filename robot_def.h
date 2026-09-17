#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "lib_math.h"

// gimbal限位(rad)
#define pitchup_position_3 1.7082574367523193f   // 下pitch直立，上pitch最大仰角
#define pitchup_position_2 0.94192671775817871f  // 下pitch直立，上pitch水平
#define pitchup_position_1 0.48469734191894531f  // 下pitch直立，上pitch最大俯角
#define pitchup_position_0 -0.11059212684631348f // 下pitch倒下，上pitch倒下

#define pitchdown_position_max 1.0266802310943604f     // 直立
#define pitchdown_position_min -0.0045540332794189453f // 倒下

// gimbal速度(rad/s)，加速度(rad/s^2)
#define pitch_speed 10.0f
#define pitch_acceleration 10.0f
#define yaw_speed 10.0f
#define yaw_acceleration 10.0f

// 底盘平动，旋转速度(m/s,rad/s)
#define chassis_translate_speed 4.0f // 摇杆通道满量程对应的平动速度
#define chassis_rotate_speed 2.0f    // gyro(小陀螺)档的自转角速度，同时是 w 的限幅

/* normal/hole 档底盘跟随云台的比例系数 [1/s]：
 * w = chassis_follow_kp * wrap(云台相对底盘的 yaw 角)，把云台相对底盘的角度拉回 0，
 * 即"硬跟随"的转动部分（平移方向部分见下面 chassis_gimbal_offset 与 app_cmd 的向量旋转）。
 * TODO 实车标定：先架空轮子确认转向（反向则改为负值），再从 1 左右往上加到跟得上又不抖。 */
#define chassis_follow_kp -30.0f

/* 跟随死区 (rad)：|云台相对底盘角 θ| 小于该值时不再跟随（w 给 0）。
 * 用途：云台停在某个角度不动时，编码器噪声 + 传动回差会让 θ 在 0 附近来回抖，
 * 跟随环就一直给底盘一个微小 w，车身在原地来回蹭。死区内直接不跟随，底盘彻底站住。
 * 死区只作用在 w 上；摇杆向量的旋转仍按真实 θ 补偿，所以平移方向不受影响。
 * TODO 实车标定：取 1~3°（0.02~0.05 rad），以云台静止时车身不抖为准。 */
#define chassis_follow_deadzone DEG_TO_RAD(10.0f)

/* 云台指向相对底盘前进方向的机械零位偏置 (rad)，逆时针为正。
 * 底盘前进方向由底盘侧自己定义好了：就是 HalfRudderInverse 的车体系 x 轴，
 * 舵轮零位已由 half_rudder_chassis/robot_def.h 的 RUDDER_ZERO_OFFSET_L/R_DEG 标定到正前方。
 * 所以云台这边只剩这一个装配偏置：yaw 电机编码器零位与"云台指向 = 底盘前进方向"
 * 对不齐，差的就是这个角。
 * 云台相对底盘前进方向的真实夹角 θ = wrap(yaw_motor_position - chassis_gimbal_offset)。
 * 底盘的跟随转动（w）和摇杆向量的旋转都用这一个 θ，两者必须同源，否则会因为
 * 跟随停在编码器零位、而向量按真实朝向旋转而互相错开一个偏置角。
 *
 * 标定：车静止，手动把云台指向摆到与底盘前进方向一致，读 yaw 电机的 position
 * （即 gimbal2cmd 回传的 yaw_motor_position），读数就是该填的值。
 * 未标定时填 0：退化为"编码器零位即对正"。 */
#define chassis_gimbal_offset DEG_TO_RAD(-30.0f)

// 我们的flysky遥控参数
#define FS_SBUS_CH_MIN 240
#define FS_SBUS_CH_MAX 1807
#define FS_SBUS_CH_CENTER 1024

#endif // !__ROBOT_DEF_H
