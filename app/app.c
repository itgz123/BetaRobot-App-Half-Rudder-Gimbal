#include "app.h"
#include "app_cmd_gimbal.h" // gimbal2cmd / cmd2gimbal 队列句柄与数据结构
#include "app_cmd_shoot.h"  // cmd2shoot / shoot2cmd 队列句柄与数据结构
#include "app_cfg.h"
//
#include "bsp_log.h"
#include "bsp_app.h"
#include "bsp_dwt.h"
#include "lib_math.h"
#include "bsp_freertos.h"
#include "bsp_map.h"
//
#include "drv_daemon.h"
#include "drv_buzzer_simple.h"
#include "drv_vofa.h"
//
#include "app_cmd.h"
#include "app_gimbal.h"
#include "app_shoot.h"

/* 日志实例定义 */
LOG_INSTANCE_DEF(g_app_log, "app", 255); // app 层日志实例

/* 队列实例定义 */
QUEUE_INSTANCE_DEF(cmd2shoot_queue, 1, cmd2shoot_data_t);
QUEUE_INSTANCE_DEF(shoot2cmd_queue, 1, shoot2cmd_data_t);
QUEUE_INSTANCE_DEF(gimbal2cmd_queue, 1, gimbal2cmd_data_t);
QUEUE_INSTANCE_DEF(cmd2gimbal_queue, 1, cmd2gimbal_data_t);

/* 蜂鸣器实例：daemon 报警与 app 音乐共用一路 PWM，音源仲裁在 drv_buzzer_simple 内 */
BUZZER_INSTANCE_DEF(g_buzzer);

/* daemon 故障音效：无参函数，函数体里直接用本文件的蜂鸣器实例。
 * daemon 只认这个签名，不认识蜂鸣器。
 * 报警用 BUZZER_PRIO_HIGH：可打断音乐，且同一序列重复调用是幂等的
 * （daemon 离线期间每拍都调，自然变成循环报警）。 */
static void AlarmMinor(void)
{
    (void)BuzzerPlay(&g_buzzer, MUSIC_EFFECT_ERR_MINOR, MUSIC_PULSE_COUNT(MUSIC_EFFECT_ERR_MINOR), BUZZER_PRIO_HIGH, 1.0f);
}

static void AlarmMajor(void)
{
    (void)BuzzerPlay(&g_buzzer, MUSIC_EFFECT_ERR_MAJOR, MUSIC_PULSE_COUNT(MUSIC_EFFECT_ERR_MAJOR), BUZZER_PRIO_HIGH, 1.0f);
}

static void AlarmCritical(void)
{
    (void)BuzzerPlay(&g_buzzer, MUSIC_EFFECT_ERR_CRITICAL, MUSIC_PULSE_COUNT(MUSIC_EFFECT_ERR_CRITICAL), BUZZER_PRIO_HIGH, 1.0f);
}

static void AlarmFatal(void)
{
    (void)BuzzerPlay(&g_buzzer, MUSIC_EFFECT_ERR_FATAL, MUSIC_PULSE_COUNT(MUSIC_EFFECT_ERR_FATAL), BUZZER_PRIO_HIGH, 1.0f);
}

static void AlarmEmergency(void)
{
    (void)BuzzerPlay(&g_buzzer, MUSIC_EFFECT_ERR_EMERGENCY, MUSIC_PULSE_COUNT(MUSIC_EFFECT_ERR_EMERGENCY), BUZZER_PRIO_HIGH, 1.0f);
}

/* daemon 故障动作表：表必须静态存储期（DaemonInit 只保存指针） */
static const Daemon_ActionTable_s s_daemon_actions = {
    .action[DAEMON_FAULT_BUZZER_MINOR] = AlarmMinor,
    .action[DAEMON_FAULT_BUZZER_MAJOR] = AlarmMajor,
    .action[DAEMON_FAULT_BUZZER_CRITICAL] = AlarmCritical,
    .action[DAEMON_FAULT_BUZZER_FATAL] = AlarmFatal,
    .action[DAEMON_FAULT_BUZZER_EMERGENCY] = AlarmEmergency,
};

/* 队列句柄（非static，供其他模块通过 extern 访问） */
QueueHandle_t cmd2shoot_queue_handle = NULL;
QueueHandle_t shoot2cmd_queue_handle = NULL;
QueueHandle_t gimbal2cmd_queue_handle = NULL;
QueueHandle_t cmd2gimbal_queue_handle = NULL;

// 任务STACK大小
#define CMD_STACK_SIZE 1024
#define GIMBAL_STACK_SIZE 1024
#define SHOOT_STACK_SIZE 1024
#define DAEMON_STACK_SIZE 512
// 任务频率设置
#define CMD_FREQ_MS 2    // 遥控，视觉，底盘-云台
#define GIMBAL_FREQ_MS 2 // 云台
#define SHOOT_FREQ_MS 2  // 发射
/* 任务实例定义 */
TASK_INSTANCE_DEF(cmd_task, CMD_STACK_SIZE);
TASK_INSTANCE_DEF(gimbal_task, GIMBAL_STACK_SIZE);
TASK_INSTANCE_DEF(shoot_task, SHOOT_STACK_SIZE);
TASK_INSTANCE_DEF(daemon_task, DAEMON_STACK_SIZE);

/* 任务函数：周期任务框架（固定周期唤醒 + 执行耗时监控 + 超时计数） */
APP_TASK_DEF(Cmd, CMD_FREQ_MS, AppCmdRun)
APP_TASK_DEF(Gimbal, GIMBAL_FREQ_MS, AppGimbalRun)
APP_TASK_DEF(Shoot, SHOOT_FREQ_MS, AppShootRun)

/* daemon 周期主体：drv_daemon 不创建任务，由 app 提供任务时基（周期见 DAEMON_FREQ_MS） */
static void AppDaemonRun(float dt, uint64_t time_stamp)
{
    (void)dt;
    (void)time_stamp;
    DaemonTask();
    BuzzerTask(&g_buzzer); // 节拍推进与 daemon 同任务（DAEMON_FREQ_MS = 1ms）
}
APP_TASK_DEF(Daemon, DAEMON_FREQ_MS, AppDaemonRun)

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
    BuzzerRegister(&g_buzzer);
    BuzzerConfig(&g_buzzer, &(Buzzer_Config_s){.tim_e = TIM_BUZZER, .volume = 1.0f});
    DaemonInit(&s_daemon_actions);
    VofaInit();
    // app
    AppGimbalInit();
    AppCmdInit();
    AppShootInit();

    // 创建队列
    create_queue();

    // 注册任务
    TaskRegister(&daemon_task, &(Task_Init_Config_s){.func = StartDaemonTask, .priority = 0});
    TaskRegister(&cmd_task, &(Task_Init_Config_s){.func = StartCmdTask, .priority = 2});
    TaskRegister(&gimbal_task, &(Task_Init_Config_s){.func = StartGimbalTask, .priority = 2});
    TaskRegister(&shoot_task, &(Task_Init_Config_s){.func = StartShootTask, .priority = 2});
    __enable_irq(); // 开启中断
}
