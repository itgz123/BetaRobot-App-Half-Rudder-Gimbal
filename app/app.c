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
#include "app_shoot.h"

/* 日志实例定义 */
LOG_INSTANCE_DEF(g_app_log, "app", 0); // app 层日志实例

/* 队列实例定义 */
QUEUE_INSTANCE_DEF(cmd2shoot_queue, 1, cmd2shoot_data_t);
QUEUE_INSTANCE_DEF(shoot2cmd_queue, 1, shoot2cmd_data_t);
QUEUE_INSTANCE_DEF(gimbal2cmd_queue, 1, gimbal2cmd_data_t);
QUEUE_INSTANCE_DEF(cmd2gimbal_queue, 1, cmd2gimbal_data_t);

/* 队列句柄（非static，供其他模块通过 extern 访问） */
QueueHandle_t cmd2shoot_queue_handle = NULL;
QueueHandle_t shoot2cmd_queue_handle = NULL;
QueueHandle_t gimbal2cmd_queue_handle = NULL;
QueueHandle_t cmd2gimbal_queue_handle = NULL;

// 任务STACK大小
#define CMD_STACK_SIZE 1024
#define GIMBAL_STACK_SIZE 1024
#define SHOOT_STACK_SIZE 1024
// 任务频率设置
#define CMD_FREQ_MS 2    // 遥控，视觉，底盘-云台
#define GIMBAL_FREQ_MS 2 // 云台
#define SHOOT_FREQ_MS 2  // 发射
/* 任务实例定义 */
TASK_INSTANCE_DEF(cmd_task, CMD_STACK_SIZE);
TASK_INSTANCE_DEF(gimbal_task, GIMBAL_STACK_SIZE);
TASK_INSTANCE_DEF(shoot_task, SHOOT_STACK_SIZE);

ITCM_RAM static __attribute__((noreturn)) void StartCmdTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    TickType_t xLastWakeTime = xTaskGetTickCount();        // 周期锚点(绝对唤醒时刻)
    const TickType_t xPeriod = pdMS_TO_TICKS(CMD_FREQ_MS); // 任务周期(tick)
    BSPLOG(&g_app_log, LOG_LEVEL_INFO, "CMD Task Start");
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod); // 固定周期唤醒，避免 vTaskDelay 的周期漂移
        start = DWT_GetTimeUs();
        AppCmdRun();
        dt = DWT_GetTimeUs() - start;
        if (dt > 1000 * CMD_FREQ_MS)
            BSPLOG(&g_app_log, LOG_LEVEL_ERROR, "CMD Task is being DELAY! dt = %llu(us)", dt);
    }
}

ITCM_RAM static __attribute__((noreturn)) void StartGimbalTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    TickType_t xLastWakeTime = xTaskGetTickCount();           // 周期锚点(绝对唤醒时刻)
    const TickType_t xPeriod = pdMS_TO_TICKS(GIMBAL_FREQ_MS); // 任务周期(tick)
    BSPLOG(&g_app_log, LOG_LEVEL_INFO, "GIMBAL Task Start");
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod); // 固定周期唤醒，避免 vTaskDelay 的周期漂移
        start = DWT_GetTimeUs();
        AppGimbalRun();
        dt = DWT_GetTimeUs() - start;
        if (dt > 1000 * GIMBAL_FREQ_MS)
            BSPLOG(&g_app_log, LOG_LEVEL_ERROR, "GIMBAL Task is being DELAY! dt = %llu(us)", dt);
    }
}

ITCM_RAM static __attribute__((noreturn)) void StartShootTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    TickType_t xLastWakeTime = xTaskGetTickCount();          // 周期锚点(绝对唤醒时刻)
    const TickType_t xPeriod = pdMS_TO_TICKS(SHOOT_FREQ_MS); // 任务周期(tick)
    BSPLOG(&g_app_log, LOG_LEVEL_INFO, "SHOOT Task Start");
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod); // 固定周期唤醒，避免 vTaskDelay 的周期漂移
        start = DWT_GetTimeUs();
        AppShootRun();
        dt = DWT_GetTimeUs() - start;
        if (dt > 1000 * SHOOT_FREQ_MS)
            BSPLOG(&g_app_log, LOG_LEVEL_ERROR, "SHOOT Task is being DELAY! dt = %llu(us)", dt);
    }
}

static void create_queue(void)
{
    cmd2shoot_queue_handle = QueueRegister(&cmd2shoot_queue);
    shoot2cmd_queue_handle = QueueRegister(&shoot2cmd_queue);
    gimbal2cmd_queue_handle = QueueRegister(&gimbal2cmd_queue);
    cmd2gimbal_queue_handle = QueueRegister(&cmd2gimbal_queue);
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
    AppGimbalInit();
    AppCmdInit();
    AppShootInit();

    // 创建队列
    create_queue();

    // 注册任务
    TaskRegister(&cmd_task, &(Task_Init_Config_s){.func = StartCmdTask, .priority = 2});
    TaskRegister(&gimbal_task, &(Task_Init_Config_s){.func = StartGimbalTask, .priority = 2});
    TaskRegister(&shoot_task, &(Task_Init_Config_s){.func = StartShootTask, .priority = 2});
    __enable_irq(); // 开启中断
}
