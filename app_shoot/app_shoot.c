/**
 * @file app_shoot.c
 * @brief 发射机构 App（当前为通信框架 UART 测试）
 *
 * 通信链路测试（comm 框架 media + proto 空协议 + 顶层分发）：
 *   - COMM_DEF 定义一条对话：media(DM_MC02: UART_10, USART10, 921600) + 接收/发送
 *     协议(空协议，payload 占 100%)，收发 payload 大小可不同，缓冲按方向自动推导
 *   - 两段式接口：CommRegister（不可重入，注册 + 接线分发）
 *               → CommConfig（可重入，介质参数 + 出帧回调，均可反复修改）
 *   - 发送：AppShootRun 每 100ms 发一帧（CommSend，走发送协议）
 *
 * @note 测试方式（硬件回环）：把 UART_10 的 TX/RX 短接，串口助手可看到
 *       板子周期发帧；助手回发的数据也会被识别并分发到 CommTestOnFrame。
 */

#include "app_shoot.h"
#include "app_cfg.h"
//
#include "bsp_uart_log.h"
#include "bsp_usart.h"
//
#include "drv_comm.h"
#include "comm_media_usart.h"
#include "comm_proto_raw.h"

#define COMM_TEST_RX_SIZE 8         /* 接收 payload 大小（空协议，帧长 = payload） */
#define COMM_TEST_TX_SIZE 8         /* 发送 payload 大小 */
#define COMM_TEST_FRAME_HEADER 0xAA /* 首字节，用于接收侧识别本测试帧 */

/* 一条对话：接收 PROTO_RAW（8B） + 发送 PROTO_RAW（8B），收发可不同，ISR 直解 */
COMM_DEF(uart_comm, MEDIA_USART, PROTO_RAW, PROTO_RAW, COMM_TEST_RX_SIZE, COMM_TEST_TX_SIZE, UNPACK_IN_ISR);

static void CommTestOnFrame(const uint8_t *payload)
{
    (void)payload; /* 目前只测试发送，还没消费接收 */
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

    /* 构造递增 payload：{0xAA, seq, seq+1, ..., seq+6} */
    payload[0] = COMM_TEST_FRAME_HEADER;
    for (int i = 0; i < COMM_TEST_TX_SIZE - 1; i++)
        payload[i + 1] = COMM_TEST_FRAME_HEADER + 1;

    CommSend(&uart_comm, payload);
}
