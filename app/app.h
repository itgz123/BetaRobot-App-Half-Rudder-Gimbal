#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/* 任务创建函数 */
void function_in_main_c(void);

// 任务间通信
#include <stdint.h>

typedef enum : uint8_t
{
    stop = 0, // 急停
    normal,   // 普通
    gyro,     // 小陀螺
} chassis_mode_e;

typedef struct
{
    float vx;
    float vy;
    float w;
    chassis_mode_e mode;
} cmd2chassis_data_t;

typedef struct
{
    float pitch;
    float yaw;
} cmd2gimbal_data_t;

typedef struct
{
    float yaw_rate;  /* yaw 角速度 (rad/s) */
    float yaw_angle; /* yaw 角度 (rad)     */
    float yaw_acc;   /* 加速度 (m/s²)      */
} sensor2chassis_data_t;

typedef struct
{
    float gyro[3];
    float acc[3];
} sensor2gimbal_data_t;

/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t cmd2chassis_queue_handle;
extern QueueHandle_t cmd2gimbal_queue_handle;
extern QueueHandle_t sensor2chassis_queue_handle;
extern QueueHandle_t sensor2gimbal_queue_handle;

#endif // !__APP_H
