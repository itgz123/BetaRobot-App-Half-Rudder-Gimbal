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
#define chassis_translate_speed 6.0f       // 平动最大速度：速度旋钮打到最大档时，摇杆通道满量程对应的平动速度
#define chassis_translate_speed_min 1.0f   // 平动最小速度：速度旋钮打到最小档时，摇杆通道满量程对应的平动速度
#define chassis_gyro_rotate_speed 12.0f    // 小陀螺最大转速：旋转速度旋钮打到最大档时，gyro(小陀螺)档底盘的自转角速度
#define chassis_gyro_rotate_speed_min 1.0f // 小陀螺最小转速：旋转速度旋钮打到最小档时的自转角速度
#define chassis_follow_w_limit 4.0f        // normal/hole 档跟随 w（比例项+前馈之和）的限幅

/* normal/hole 档底盘跟随云台的比例系数 [1/s]：
 * w = chassis_follow_kp * wrap(云台相对底盘的 yaw 角) + 前馈项（见下面 chassis_follow_ff），
 * 把云台相对底盘的角度拉回 0，即"硬跟随"的转动部分
 * （平移方向部分见下面 chassis_gimbal_offset 与 app_cmd 的向量旋转）。
 * TODO 实车标定：先架空轮子确认转向（反向则改为负值），再从 1 左右往上加到跟得上又不抖。 */
#define chassis_follow_kp -3.0f

/* normal/hole 档底盘跟随云台的速度前馈系数（无量纲，理想值 1.0）。
 *
 * 只有比例项时，操作手一推 yaw 摇杆，云台先转（yaw 轴锁世界系航向），底盘要等
 * "云台相对底盘的角度 θ"建立起误差之后才跟着转，起步慢半拍。
 * 前馈把云台自己的指令角速度 ω_yaw 直接加到底盘 w 上：
 *     w = ff * ω_yaw + kp * wrap(θ)
 * ω_yaw 与底盘 w 都是"角速度"量，方向天然一致，ff 给 1.0 即可让 ff*ω_yaw 抵消云台转动
 * 带来的 θ 变化：底盘与云台同步转，θ 不再被拉开，kp 只负责收拾残余误差。
 *   |ff| = 1.0：完全前馈，底盘与云台指令同速（推荐起点）
 *   |ff| < 1  ：保留少量跟随滞后；|ff| > 1 或标定失真会超调/画龙
 * 符号不是推导出来的而是实车定的：本车的关节角/世界系航向实际符号与注释里的
 * "逆时针为正"相反（kp 也是因此取的负），所以 ff 同样取负。标定时和 chassis_follow_kp
 * 一起看：两者必须同号，否则前馈是在帮倒忙（把 θ 越推越大）。 */
#define chassis_follow_ff -1.0f

/* 跟随死区 (rad)：|云台相对底盘角 θ| 小于该值时不再跟随（比例项给 0）。
 * 用途：云台停在某个角度不动时，编码器噪声 + 传动回差会让 θ 在 0 附近来回抖，
 * 跟随环就一直给底盘一个微小 w，车身在原地来回蹭。死区内直接不跟随，底盘彻底站住。
 * 只屏蔽比例项，不屏蔽前馈项：前馈是云台"正在转"时才有的量，云台静止时它本来就是 0，
 * 而且要的正是"误差还没建立起来就先转"。
 * 死区只作用在 w 上；摇杆向量的旋转仍按真实 θ 补偿，所以平移方向不受影响。
 * TODO 实车标定：取 1~3°（0.02~0.05 rad），以云台静止时车身不抖为准。 */
#define chassis_follow_deadzone DEG_TO_RAD(2.0f)

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

/* 拨弹机构机械参数
 * 拨盘一圈 12 个孔，相邻两孔夹角 360/12 = 30°，每转过 30° 送出一颗弹；
 * 电机与拨盘减速比 25.6（电机转 25.6 圈，拨盘转 1 圈），故每发一颗电机需转过
 *     TRIGGER_STEP_RAD = 2π × 25.6 / 12 ≈ 13.404 rad（2.133 圈）
 * 该值是纯机械量，实车换拨盘/减速箱后只改这里；速度/加速度/电流等控制参数仍在 app_shoot.c。*/
#define TRIGGER_PADDLE_HOLES 12                                                    // 拨盘一圈孔数（= 一圈的弹数）
#define TRIGGER_GEAR_RATIO 25.6f                                                   // 电机:拨盘 减速比
#define TRIGGER_STEP_RAD (2.0f * M_PI * TRIGGER_GEAR_RATIO / TRIGGER_PADDLE_HOLES) // 每发一颗电机转角 (rad)

/* 摩擦轮机械参数与弹速上限
 * 摩擦轮直径 5cm（半径 2.5cm），弹丸出口速度 ≈ 摩擦轮线速度：v = ω·r。
 * 规则弹丸速度上限 25 m/s，实际留余量取 22 m/s（约 12% 余量，覆盖弹丸批次差异与
 * 摩擦轮磨损/打滑）。22 m/s 对应摩擦轮 880 rad/s（≈8400 rpm）：若摩擦轮与电机
 * 反馈轴之间还有减速，把 FRICTION_REDUCTION 填成该传动比（电机:摩擦轮 = n:1 时
 * ω_电机 = n·v/r）；摩擦轮直接装在反馈轴上则填 1。
 * ch9 调试旋钮 -1~1 映射到 0~BULLET_SPEED_MAX (m/s)，换算在 app_shoot.c。 */
#define FRICTION_WHEEL_DIAMETER 0.05f                          // 摩擦轮直径 (m)
#define FRICTION_WHEEL_RADIUS (FRICTION_WHEEL_DIAMETER * 0.5f) // 摩擦轮半径 (m) = 0.025
#define FRICTION_REDUCTION 1.0f                                // 电机反馈轴:摩擦轮 传动比（直驱 = 1）
#define BULLET_SPEED_RULE_MAX 25.0f                            // 规则弹丸速度上限 (m/s)
#define BULLET_SPEED_MAX 22.0f                                 // 实际使用弹速上限 (m/s)，ch9 映射上限

// 我们的flysky遥控参数
#define FS_SBUS_CH_MIN 240
#define FS_SBUS_CH_MAX 1807
#define FS_SBUS_CH_CENTER 1024

#endif // !__ROBOT_DEF_H
