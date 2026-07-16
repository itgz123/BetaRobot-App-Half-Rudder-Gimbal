#include "app.h"
#include "app_cfg.h"
//
#include "bsp_uart_log.h"
#include "bsp_dwt.h"
#include "bsp_math.h"
#include "bsp_freertos.h"
#include "bsp_map.h"
//
#include "drv_chassis_position_lite.h"
#include "drv_daemon.h"
#include "drv_vofa.h"
//
#include "app_cmd.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_sensor.h"

/* 队列实例定义 */
QUEUE_INSTANCE_DEF(cmd2chassis_queue, 1, cmd2chassis_data_t);
QUEUE_INSTANCE_DEF(cmd2gimbal_queue, 1, cmd2gimbal_data_t);
QUEUE_INSTANCE_DEF(sensor2chassis_queue, 1, sensor2chassis_data_t);
QUEUE_INSTANCE_DEF(sensor2gimbal_queue, 1, sensor2gimbal_data_t);

/* 队列句柄（非static，供其他模块通过 extern 访问） */
QueueHandle_t cmd2chassis_queue_handle = NULL;
QueueHandle_t cmd2gimbal_queue_handle = NULL;
QueueHandle_t sensor2chassis_queue_handle = NULL;
QueueHandle_t sensor2gimbal_queue_handle = NULL;

// 任务STACK大小
#define CMD_STACK_SIZE 512
#define CHASSIS_STACK_SIZE 512
#define GIMBAL_STACK_SIZE 512
#define SENSOR_STACK_SIZE 512
#define ERROR_STACK_SIZE 512
// 任务频率设置
#define CMD_FREQ_MS 2     // 遥控
#define CHASSIS_FREQ_MS 2 // 底盘
#define GIMBAL_FREQ_MS 2  // 云台
#define SENSOR_FREQ_MS 1  // 传感器
/* 任务实例定义 */
TASK_INSTANCE_DEF(cmd_task, CMD_STACK_SIZE);
TASK_INSTANCE_DEF(chassis_task, CHASSIS_STACK_SIZE);
TASK_INSTANCE_DEF(gimbal_task, GIMBAL_STACK_SIZE);
TASK_INSTANCE_DEF(sensor_task, SENSOR_STACK_SIZE);

ITCM_RAM static __attribute__((noreturn)) void StartChassisTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    LOGINFO("[freeRTOS] CHASSIS Task Start");
    for (;;)
    {
        start = DWT_GetTimeUs();
        AppChassisRun();
        dt = DWT_GetTimeUs() - start;
        if ((dt / 1000) > CHASSIS_FREQ_MS)
            LOGERROR("[freeRTOS] CHASSIS Task is being DELAY! dt = %d(ms)", (dt / 1000));
        vTaskDelay(pdMS_TO_TICKS(CHASSIS_FREQ_MS));
    }
}

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

static void create_queue(void)
{
    cmd2chassis_queue_handle = QueueRegister(&cmd2chassis_queue);
    cmd2gimbal_queue_handle = QueueRegister(&cmd2gimbal_queue);
    sensor2chassis_queue_handle = QueueRegister(&sensor2chassis_queue);
    sensor2gimbal_queue_handle = QueueRegister(&sensor2gimbal_queue);
}

void function_in_main_c(void)
{
    __disable_irq();
    BSPInit();
    DWT_Init();
    __enable_irq();

    // 初始化
    BSPLogInit();
    DaemonInit();
    VofaInit();

    AppSensorInit();
    AppGimbalInit();
    AppChassisInit();
    AppCmdInit();

    // 创建队列
    create_queue();

    // 注册任务
    TaskRegister(&cmd_task, &(Task_Init_Config_s){.func = StartCmdTask, .priority = 1});
    TaskRegister(&chassis_task, &(Task_Init_Config_s){.func = StartChassisTask, .priority = 1});
    TaskRegister(&gimbal_task, &(Task_Init_Config_s){.func = StartGimbalTask, .priority = 1});
    TaskRegister(&sensor_task, &(Task_Init_Config_s){.func = StartSensorTask, .priority = 3});
}
