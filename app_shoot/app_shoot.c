/**
 * @file app_shoot.c
 * @brief 发射机构 App（当前为通信框架 UART 测试）
 *
 * 通信链路测试（comm 框架 media + proto 空协议 + 顶层分发）：
 *   - COMM_DEF 定义一条对话：media(DM_MC02: UART_10, USART10, 921600) + 接收/发送
 *     协议(PROTO_CUSTOM：帧头/帧尾/seq/CRC8)，收发 payload 大小可不同，缓冲按方向自动推导
 *   - 两段式接口：CommRegister（不可重入，注册 + 接线分发）
 *               → CommConfig（可重入，介质参数 + 出帧回调，均可反复修改）
 *   - 发送：AppShootRun 每 100ms 发一帧（CommSend，走发送协议）
 *
 * @note 测试方式（硬件回环）：把 UART_10 的 TX/RX 短接，串口助手可看到
 *       板子周期发帧（帧格式 A5 [seq] [payload] [crc] 5A）；助手回发的数据
 *       也会被识别（帧头/帧尾/CRC8/seq 校验）并分发到 CommTestOnFrame。
 */

#include "app_shoot.h"
#include "app_cfg.h"
//
#include "bsp_uart_log.h"
#include "bsp_usart.h"
//
#include "drv_comm.h"
#include "comm_media_usart.h"
#include "comm_proto_custom.h"

uint16_t test = 0;

#define COMM_TEST_RX_SIZE 8 /* 接收 payload 大小（PROTO_CUSTOM，帧长 = payload + 4） */
#define COMM_TEST_TX_SIZE 8 /* 发送 payload 大小 */

/* 一条对话：接收 PROTO_CUSTOM（8B） + 发送 PROTO_CUSTOM（8B），收发可不同，ISR 直解 */
COMM_DEF(uart_comm, MEDIA_USART, PROTO_CUSTOM, PROTO_CUSTOM, COMM_TEST_RX_SIZE, COMM_TEST_TX_SIZE, UNPACK_IN_ISR);

static void CommTestOnFrame(const uint8_t *payload)
{
    /* 打印收到的帧序列（回环下 = 本地发送 seq，连续递增验证 seq/丢帧检测） */
    LOGINFO("[app_shoot] rx seq=%u", (unsigned)payload[0]);
    test++;
}

void AppShootInit(void)
{
    /* 单个 config：介质参数 + 出帧回调整合（可重入，运行期可改） */
    CommConfig_s cfg = {
        .media_cfg = &(USART_Config_s){
            .uart_e = UART_10,
            .tx_mode = USART_DMA_MODE,
        },
        .on_frame = CommTestOnFrame,
    };

    /* 两段式：注册（不可重入）→ 配置（可重入） */
    CommRegister(&uart_comm);
    CommConfig(&uart_comm, &cfg);
}

void AppShootRun(void)
{
    uint8_t payload[COMM_TEST_TX_SIZE];

    /* 构造递增 payload（协议层负责加帧头/seq/CRC8/帧尾；payload 首字节可当发送 seq 观察） */
    for (int i = 0; i < COMM_TEST_TX_SIZE; i++)
        payload[i] = (uint8_t)i;

    CommSend(&uart_comm, payload);
}
