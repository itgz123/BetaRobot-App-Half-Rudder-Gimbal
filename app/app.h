#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/* 任务创建函数 */
void function_in_main_c(void);

// 任务间通信
#include <stdint.h>

typedef enum : uint8_t
{
    disable = 0,
    enable = 1,
} cmd2gimbal_state; // 状态机

typedef struct
{
    uint8_t temp_unused;
} cmd2shoot_data_t;
typedef struct
{
    uint8_t temp_unused;
} shoot2cmd_data_t;
typedef struct
{
    uint8_t temp_unused;
} gimbal2cmd_data_t;
typedef struct
{
    cmd2gimbal_state state; // 状态机
} cmd2gimbal_data_t;
typedef struct
{
    uint8_t temp_unused;
} sensor2gimbal_data_t;
typedef struct
{
    uint8_t temp_unused;
} gimbal2sensor_data_t;
/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t cmd2shoot_queue_handle;
extern QueueHandle_t shoot2cmd_queue_handle;
extern QueueHandle_t gimbal2cmd_queue_handle;
extern QueueHandle_t cmd2gimbal_queue_handle;
extern QueueHandle_t sensor2gimbal_queue_handle;
extern QueueHandle_t gimbal2sensor_queue_handle;

#endif // !__APP_H
