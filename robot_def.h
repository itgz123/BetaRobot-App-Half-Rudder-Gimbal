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
#define pitch_speed 3.0f
#define pitch_acceleration 10.0f
#define yaw_speed 3.0f
#define yaw_acceleration 10.0f

// 底盘平动，旋转速度(m/s,rad/s)
#define chassis_translate_speed 6.0f       // 平动速度上限：档位 +1（原速度旋钮最大档）时的满舵平动速度
#define chassis_translate_speed_min 1.0f   // 平动速度下限：档位 -1（原速度旋钮最小档）时的满舵平动速度
#define chassis_gyro_rotate_speed 12.0f    // 小陀螺转速上限：档位 +1 时 gyro 档底盘的自转角速度
#define chassis_gyro_rotate_speed_min 1.0f // 小陀螺转速下限：档位 -1 时的自转角速度
/* 原 ch8 速度旋钮已改作视觉开关，档位固定为 0 ⇒ 上述两组上下限都取中点。 */

/* normal/hole 档底盘跟随云台的速度前馈系数（无量纲，理想值 1.0）。
 *
 * 底盘跟随律拆成两半：
 *   云台侧（本宏）：w_ff = chassis_follow_ff * ω_yaw，随帧发 w_ff 与目标偏角 θ_ref
 *   底盘侧        ：自己读共享 CAN1 的 RS05 得到实测 θ，
 *                   w = w_ff + PID(wrap(θ − θ_ref))，再统一限幅
 * PID/死区/限幅参数都在底盘 app_chassis_force_def.h 的 CHASSIS_FOLLOW_*。
 * θ_ref 在 normal/hole 为 0，gyro 按 −自转速度斜坡变化（见 app_cmd.c）。
 *
 * 只有比例项时，操作手一推 yaw 摇杆，云台先转（yaw 轴锁世界系航向），底盘要等
 * "云台相对底盘的角度 θ"建立起误差之后才跟着转，起步慢半拍。
 * 前馈把云台自己的指令角速度 ω_yaw 直接加到底盘 w 上：ω_yaw 与底盘 w 都是
 * "角速度"量，方向天然一致，ff 给 1.0 即可让 ff*ω_yaw 抵消云台转动带来的 θ 变化：
 * 底盘与云台同步转，θ 不再被拉开，kp 只负责收拾残余误差。
 *   |ff| = 1.0：完全前馈，底盘与云台指令同速（推荐起点）
 *   |ff| < 1  ：保留少量跟随滞后；|ff| > 1 或标定失真会超调/画龙
 * 符号：ω_yaw 与底盘 w 同是"逆时针为正"，ff 必须为正。
 * 标定时和底盘侧 PID 一起看：前馈与 PID 必须同号，否则是在帮倒忙（把 θ 越推越大）。 */
#define chassis_follow_ff 1.0f

/* 云台指向相对底盘前进方向的机械零位偏置 (rad)，逆时针为正。
 * 底盘前进方向由底盘侧自己定义好了：就是 HalfRudderInverse 的车体系 x 轴，
 * 舵轮零位已由 half_rudder_chassis/robot_def.h 的 RUDDER_ZERO_OFFSET_L/R_DEG 标定到正前方。
 * 所以云台这边只剩这一个装配偏置：yaw 电机编码器零位与"云台指向 = 底盘前进方向"
 * 对不齐，差的就是这个角。
 * 云台相对底盘前进方向的真实夹角 θ = wrap(yaw_motor_position - chassis_gimbal_offset)。
 * 云台侧用它旋转摇杆向量；底盘侧直接读同一颗 RS05 的原始反馈，扣同一个偏置
 * （chassis 的 CHASSIS_GIMBAL_OFFSET，数值必须与本宏一致）得到自己的实测 θ。
 * 帧里的 gimbal2chassis_data.yaw 是目标偏角 θ_ref，不是实测角。
 *
 * 标定：车静止，手动把云台指向摆到与底盘前进方向一致，读 yaw 电机的 position
 * （即 gimbal2cmd 回传的 yaw_motor_position），读数就是该填的值。
 * 未标定时填 0：退化为"编码器零位即对正"。 */
#define chassis_gimbal_offset DEG_TO_RAD(30.0f)

/* 摇杆向量旋转角的执行延迟补偿时间 T (s)，只作用于 send_chassis 的向量旋转。
 *
 * 从 yaw_motor_position 采样到本拍 (vx,vy) 真正被底盘执行，中间有：
 *   云台反馈年龄（两任务均 2ms，约 2~4ms）
 * + CAN 三包 + 底盘 2ms 任务量化 + 底盘输出一拍滞后（约 2~5ms）。
 * 小陀螺档云台锁世界系航向、底盘以实测 w>0 逆时针自转时 dθ/dt = -w，
 * 故执行时刻的预测角是
 *   theta_vec = wrap(theta - w * T)
 * 代码里是减号：不补偿时平移方向会比瞄准线逆时针偏 wT。
 *
 * 0 = 关闭补偿（行为与改动前完全一致）；建议从 0.004~0.006 起调，按 1ms 步进。
 * gyro 档推前，若轨迹仍偏逆时针（左）→ 加大；偏顺时针（右）→ 减小。
 * 只改向量旋转；发给底盘的跟随角度仍是未补偿的 theta。 */
#define chassis_vec_theta_delay_s 0.02f

/* 拨弹机构机械参数
 * 拨盘一圈 12 个孔，相邻两孔夹角 360/12 = 30°，每转过 30° 送出一颗弹；
 * 电机与拨盘是**两级**减速 268:17 × 62:38 = 25.7214（电机转 25.7214 圈，拨盘转 1 圈），
 * 故每发一颗电机需转过
 *     TRIGGER_STEP_RAD = 2π × 25.7214 / 12 ≈ 13.4677 rad（2.143 圈）
 * ⚠ 这里原写死 25.6（四舍五入值，偏小 0.474%）。后果不是"速度差点"——是**相位**：
 *   每发滑 0.474% 个孔 ⇒ **每 211 发滑掉整整一个孔**（6 发/s 连发 ≈ 35 s）。42 发的单发
 *   试验里只漂 0.21 孔，藏在"隔发一响"里看不出来；上 100 发就翻篇。做整数网格停位才暴露。
 * ✅ 实测已确认 25.7214：位置域折叠（拨盘 |CH8| 13.4693 / 摩擦轮凹陷 13.4703）与事件栅格
 *   回归 13.4699 rad，对 13.4677 的偏差只有 0.017% ⇒ 12 孔/圈 + 268:17×62:38 同时成立。
 *   参考车 half_rudder_2027 里写的 25.6 是同一个四舍五入值；它只拿它算巡航转速（那 0.474%
 *   看不出来），我们拿它做网格停位才必须用真值。
 * 该值是纯机械量，实车换拨盘/减速箱后只改这里；速度/加速度/电流等控制参数在 app_shoot_def.h。*/
#define TRIGGER_PADDLE_HOLES 12                                                    // 拨盘一圈孔数（= 一圈的弹数）
#define TRIGGER_GEAR_RATIO ((268.0f / 17.0f) * (62.0f / 38.0f))                    // 电机:拨盘 减速比 = 268:17 × 62:38 = 25.7214
#define TRIGGER_STEP_RAD (2.0f * M_PI * TRIGGER_GEAR_RATIO / TRIGGER_PADDLE_HOLES) // 每发一颗电机转角 (rad) ≈ 13.4677

/* 摩擦轮机械参数与弹速上限
 * 摩擦轮半径 23mm），弹丸出口速度 ≈ 摩擦轮线速度：v = ω·r。
 * 规则弹丸速度上限 25 m/s，实际留余量取 22 m/s（约 12% 余量，覆盖弹丸批次差异与
 * 摩擦轮磨损/打滑）。22 m/s 对应摩擦轮 880 rad/s（≈8400 rpm）：若摩擦轮与电机
 * 反馈轴之间还有减速，把 FRICTION_REDUCTION 填成该传动比（电机:摩擦轮 = n:1 时
 * ω_电机 = n·v/r）；摩擦轮直接装在反馈轴上则填 1。
 * ch9 调试旋钮 -1~1 映射到 0~BULLET_SPEED_MAX (m/s)，换算在 app_shoot_def.h。 */
#define FRICTION_WHEEL_RADIUS 0.023 // 摩擦轮半径 (m) = 0.023
#define FRICTION_REDUCTION 1.0f     // 电机反馈轴:摩擦轮 传动比（直驱 = 1）
#define BULLET_SPEED_RULE_MAX 25.0f // 规则弹丸速度上限 (m/s)
#define BULLET_SPEED_MAX 22.0f      // 实际使用弹速上限 (m/s)，ch9 映射上限

// 我们的flysky遥控参数
#define FS_SBUS_CH_MIN 240
#define FS_SBUS_CH_MAX 1807
#define FS_SBUS_CH_CENTER 1024

#endif // !__ROBOT_DEF_H
