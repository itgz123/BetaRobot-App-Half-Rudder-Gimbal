#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

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

#endif // !__ROBOT_DEF_H
