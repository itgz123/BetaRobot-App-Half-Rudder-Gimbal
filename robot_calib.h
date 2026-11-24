#ifndef __ROBOT_CALIB_H
#define __ROBOT_CALIB_H

#include "lib_math.h"

/*============================================
 *        云台 yaw 与底盘前进方向的机械零位
 *============================================*/
/* ⚠ 本文件是**每个 app 各一份**的本地副本（app/half_rudder_gimbal/robot_calib.h
 *   与 app/half_rudder_chassis/robot_calib.h）。两个 app 是独立 git 仓库，主仓库
 *   的 app/ 目录不追踪公共头文件，所以不再有跨 app 的 include。
 *   两份内容必须保持完全一致：云台侧与底盘侧用的是同一个机械零位，
 *   改这里时务必同时改另一个 app 的同名文件（改动后 diff 两份文件应为空）。 */
/* 标定（只改下面一个宏）：
 *   1. 车静止，手动把云台 yaw 摆到与底盘前进方向一致（对齐位置）；
 *   2. 读 yaw_motor.data[?].position（RS05 原始协议位置，rad，范围 ±12.57）；
 *   3. 把读数原样填到 YAW_ALIGN_MOTOR_RAW_POSITION_RAD。
 *
 * 为什么还要再算一次：
 *   yaw_motor.data[].position 是**原始 raw**；app_gimbal 的 lib_motor 配了
 *   feedback_direction = REVERSE，所以云台关节角是 wrap(-raw)。
 *   对齐时 θ = 0：
 *       云台：θ = wrap(yaw_ctrl.data.position - offset) = wrap(-raw - offset)
 *       底盘：θ = wrap(-raw - CHASSIS_GIMBAL_OFFSET)
 *   两边都要 offset = wrap(-raw)，下面的宏会自动做这个取负 + wrap，
 *   云台/底盘共用同一个值，不需要再手填 15°/30° 之类的角度。
 */
#define YAW_ALIGN_MOTOR_RAW_POSITION_RAD (5.9883709f)
/* 当前标定值折算：offset = wrap(-5.9883709) = +0.2948144 rad = +16.89°
 * （原来手调的 15° 已删除，只保留这个实测值） */

/* 对齐时的机械零位偏置 (rad)：offset = wrap(-raw)，由标定值自动计算 */
#define YAW_ALIGN_MOTOR_POSITION_RAD (Lib_Math_WrapAngleNegPIToPI(-(YAW_ALIGN_MOTOR_RAW_POSITION_RAD)))

/* 云台/底盘共用的机械零位偏置。两板各自 include 本文件。 */
#define ROBOT_CHASSIS_GIMBAL_OFFSET (YAW_ALIGN_MOTOR_POSITION_RAD)

#endif // !__ROBOT_CALIB_H
