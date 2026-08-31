#include "app.h"
#include "app_cfg.h"
//
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "bsp_math.h"
#include "bsp_freertos.h"
#include "bsp_map.h"
//
#include "drv_daemon.h"
#include "drv_vofa.h"
//
#include "app_cmd.h"
#include "app_gimbal.h"
#include "app_sensor.h"
#include "app_shoot.h"

/* 队列实例定义 */
QUEUE_INSTANCE_DEF(cmd2shoot_queue, 1, cmd2shoot_data_t);
QUEUE_INSTANCE_DEF(shoot2cmd_queue, 1, shoot2cmd_data_t);
QUEUE_INSTANCE_DEF(gimbal2cmd_queue, 1, gimbal2cmd_data_t);
QUEUE_INSTANCE_DEF(cmd2gimbal_queue, 1, cmd2gimbal_data_t);
QUEUE_INSTANCE_DEF(sensor2gimbal_queue, 1, sensor2gimbal_data_t);
QUEUE_INSTANCE_DEF(gimbal2sensor_queue, 1, gimbal2sensor_data_t);
QUEUE_INSTANCE_DEF(sensor2cmd_queue, 1, sensor2cmd_data_t);
QUEUE_INSTANCE_DEF(cmd2sensor_queue, 1, cmd2sensor_data_t);

/* 队列句柄（非static，供其他模块通过 extern 访问） */
QueueHandle_t cmd2shoot_queue_handle = NULL;
QueueHandle_t shoot2cmd_queue_handle = NULL;
QueueHandle_t gimbal2cmd_queue_handle = NULL;
QueueHandle_t cmd2gimbal_queue_handle = NULL;
QueueHandle_t sensor2gimbal_queue_handle = NULL;
QueueHandle_t gimbal2sensor_queue_handle = NULL;
QueueHandle_t sensor2cmd_queue_handle = NULL;
QueueHandle_t cmd2sensor_queue_handle = NULL;
// 任务STACK大小
#define CMD_STACK_SIZE 1024
#define GIMBAL_STACK_SIZE 1024
#define SENSOR_STACK_SIZE 1024
#define SHOOT_STACK_SIZE 1024
// 任务频率设置
#define CMD_FREQ_MS 1     // 遥控
#define GIMBAL_FREQ_MS 1  // 云台
#define SENSOR_FREQ_MS 1  // 传感器
#define SHOOT_FREQ_MS 100 // 发射
/* 任务实例定义 */
TASK_INSTANCE_DEF(cmd_task, CMD_STACK_SIZE);
TASK_INSTANCE_DEF(gimbal_task, GIMBAL_STACK_SIZE);
TASK_INSTANCE_DEF(sensor_task, SENSOR_STACK_SIZE);
TASK_INSTANCE_DEF(shoot_task, SHOOT_STACK_SIZE);

ITCM_RAM static __attribute__((noreturn)) void StartCmdTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    LOGINFO("[freeRTOS] CMD Task Start");
    for (;;)
    {
        start = DWT_GetTimeUs();
        AppCmdRun();
        dt = DWT_GetTimeUs() - start;
        if ((dt / 1000) > CMD_FREQ_MS)
            LOGERROR("[freeRTOS] CMD Task is being DELAY! dt = %d(ms)", (dt / 1000));
        vTaskDelay(pdMS_TO_TICKS(CMD_FREQ_MS));
    }
}

ITCM_RAM static __attribute__((noreturn)) void StartGimbalTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    LOGINFO("[freeRTOS] GIMBAL Task Start");
    for (;;)
    {
        start = DWT_GetTimeUs();
        AppGimbalRun();
        dt = DWT_GetTimeUs() - start;
        if ((dt / 1000) > GIMBAL_FREQ_MS)
            LOGERROR("[freeRTOS] GIMBAL Task is being DELAY! dt = %d(ms)", (dt / 1000));
        vTaskDelay(pdMS_TO_TICKS(GIMBAL_FREQ_MS));
    }
}

ITCM_RAM static __attribute__((noreturn)) void StartSensorTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    LOGINFO("[freeRTOS] SENSOR Task Start");
    for (;;)
    {
        start = DWT_GetTimeUs();
        AppSensorRun();
        dt = DWT_GetTimeUs() - start;
        if ((dt / 1000) > SENSOR_FREQ_MS)
            LOGERROR("[freeRTOS] SENSOR Task is being DELAY! dt = %d(ms)", (dt / 1000));
        vTaskDelay(pdMS_TO_TICKS(SENSOR_FREQ_MS));
    }
}

ITCM_RAM static __attribute__((noreturn)) void StartShootTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    LOGINFO("[freeRTOS] SHOOT Task Start");
    for (;;)
    {
        start = DWT_GetTimeUs();
        AppShootRun();
        dt = DWT_GetTimeUs() - start;
        if ((dt / 1000) > SHOOT_FREQ_MS)
            LOGERROR("[freeRTOS] SHOOT Task is being DELAY! dt = %d(ms)", (dt / 1000));
        vTaskDelay(pdMS_TO_TICKS(SHOOT_FREQ_MS));
    }
}

static void create_queue(void)
{
    cmd2shoot_queue_handle = QueueRegister(&cmd2shoot_queue);
    shoot2cmd_queue_handle = QueueRegister(&shoot2cmd_queue);
    gimbal2cmd_queue_handle = QueueRegister(&gimbal2cmd_queue);
    cmd2gimbal_queue_handle = QueueRegister(&cmd2gimbal_queue);
    sensor2gimbal_queue_handle = QueueRegister(&sensor2gimbal_queue);
    gimbal2sensor_queue_handle = QueueRegister(&gimbal2sensor_queue);
    sensor2cmd_queue_handle = QueueRegister(&sensor2cmd_queue);
    cmd2sensor_queue_handle = QueueRegister(&cmd2sensor_queue);
}

void function_in_main_c(void)
{
    // 初始化
    __disable_irq(); // 关闭中断
    BSPInit();
    DWT_Init();
    BSPLogInit(); // 初始化日志依赖的 bsp 外设（DWT，幂等）
    DaemonInit();
    VofaInit();
    // app
    AppSensorInit();
    AppGimbalInit();
    AppCmdInit();
    AppShootInit();

    // 创建队列
    create_queue();

    // 注册任务
    TaskRegister(&cmd_task, &(Task_Init_Config_s){.func = StartCmdTask, .priority = 2});
    TaskRegister(&gimbal_task, &(Task_Init_Config_s){.func = StartGimbalTask, .priority = 2});
    TaskRegister(&sensor_task, &(Task_Init_Config_s){.func = StartSensorTask, .priority = 3});
    TaskRegister(&shoot_task, &(Task_Init_Config_s){.func = StartShootTask, .priority = 2});
    __enable_irq(); // 开启中断
}
