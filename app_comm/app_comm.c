/**
 * @file app_comm.c
 * @brief 通信框架应用接线（阶段 1 最小闭环示例）
 *
 * 链路：上位机串口(huart1) -> CommMediaUsart -> Engine ring -> RX 任务
 *       -> CommProtoCustom 解包 -> 按 comm_id 分发 -> 测试队列 -> 测试任务。
 *
 * 用法示例（主机侧构造 0x5A 帧）：
 *   [0x5A][comm_id=0x01][len=0x08 0x00][seq][crc8][data(8B)][crc16 lo][crc16 hi]
 */

#include "app_comm.h"
#include "app_cfg.h"
//
#include "bsp_freertos.h"
#include "bsp_uart_log.h"
//
#include "drv_comm.h"
#include "media/comm_media_usart.h"
#include "proto/comm_proto_custom.h"

/* 实例定义（DEF 宏：同时定义派生实例 _child + 基类指针） */
COMM_MEDIA_DEF(comm_uart1, MEDIA_USART);
COMM_PROTO_DEF(comm_custom, PROTO_CUSTOM);

/* 测试消费者队列：comm_id=1 的定长载荷 */
typedef struct
{
    uint8_t data[8];
} comm_test_msg_t;
QUEUE_INSTANCE_DEF(comm_test_queue, 8, comm_test_msg_t);
static QueueHandle_t s_test_queue_h = NULL; /* 由 AppCommInit 经 QueueRegister 获取 */

/* RX 任务 */
TASK_INSTANCE_DEF(comm_rx_task, 512);

static void StartCommRxTask(void *argument)
{
    (void)argument;
    LOGINFO("[app_comm] COMM RX Task Start");
    EngineRxTask(); /* 内部自带 for(;;) */
}

/* 回环测试任务：收 comm_id=1 载荷 -> LOGINFO -> EngineSend 原样回发（验 pack/unpack 对称性） */
TASK_INSTANCE_DEF(comm_echo_task, 512);

static void StartCommEchoTask(void *argument)
{
    (void)argument;
    LOGINFO("[app_comm] COMM Echo Task Start");
    comm_test_msg_t msg;
    for (;;)
    {
        if (xQueueReceive(s_test_queue_h, &msg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            LOGINFO("[app_comm] rx comm_id=1 len=%d", (int)sizeof(msg.data));
            EngineSend(1, 1, msg.data, sizeof(msg.data)); /* 回环 */
        }
    }
}

void AppCommInit(void)
{
    EngineInit();

    /* 1. 介质：UART1 后端 */
    EngineAttachMedia(comm_uart1);
    CommMediaUsart_Config_s media_cfg = {
        .uart_e = UART_1,
        .tx_mode = USART_DMA_MODE,
        .media_id = 1,
        .unpack_in_isr = 0,
    };
    MediaUsartConfig(comm_uart1, &media_cfg);

    /* 2. 协议：统一自定义帧（0x5A），挂到该介质 */
    CommProtoCustom_Config_s proto_cfg = {
        .proto_id = 1,
        .max_payload = sizeof(comm_test_msg_t),
        .daemon = NULL,
    };
    CommProtoCustomConfig(comm_custom, &proto_cfg);
    EngineAttachProtocol(comm_uart1, comm_custom);

    /* 3. 消费者：comm_id=1 的载荷进测试队列 */
    s_test_queue_h = QueueRegister(&comm_test_queue);
    EngineRegisterConsumer(&(EngineConsumer_Config_s){
        .type = ENGINE_CONSUMER_QUEUE,
        .comm_id = 1,
        .queue = s_test_queue_h,
        .item_size = sizeof(comm_test_msg_t),
    });

    /* 4. 接收任务（优先级 4，高于 4 个业务任务） */
    TaskRegister(&comm_rx_task, &(Task_Init_Config_s){.func = StartCommRxTask, .priority = 4});

    /* 5. 回环测试任务（同优先级，仅阶段 1 验证用，可整体删掉） */
    TaskRegister(&comm_echo_task, &(Task_Init_Config_s){.func = StartCommEchoTask, .priority = 4});

    LOGINFO("[app_comm] DRV_COMM initialized");
}
