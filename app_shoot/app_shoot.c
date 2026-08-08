/**
 * @file app_shoot.c
 * @brief 发射机构 App（当前为通信框架 UART 测试）
 *
 * 通信链路测试（comm 框架 media + proto 空协议 + 顶层分发）：
 *   - COMM_DEF 合并定义 media(DM_MC02: UART_10, USART10, 921600) + proto(空协议，
 *     payload 占 100%)，缓冲区大小由 payload_size 自动推导
 *   - 两段式接口：CommRegister（不可重入，注册 + 接线分发）
 *               → CommConfig（可重入，介质参数 + 出帧回调，均可反复修改）
 *   - 发送：AppShootRun 每 100ms 发一帧（CommSend）
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

#define COMM_TEST_PAYLOAD_SIZE 8    /* payload 固定长度（空协议，帧长 = payload） */
#define COMM_TEST_FRAME_HEADER 0xAA /* 首字节，用于接收侧识别本测试帧 */

COMM_DEF(uart_comm, MEDIA_USART, PROTO_RAW, COMM_TEST_PAYLOAD_SIZE);

static void CommTestOnFrame(CommProto *proto, const uint8_t *payload)
{
    (void)proto;
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
    uint8_t payload[COMM_TEST_PAYLOAD_SIZE];

    /* 构造递增 payload：{0xAA, seq, seq+1, ..., seq+6} */
    payload[0] = COMM_TEST_FRAME_HEADER;
    for (int i = 0; i < COMM_TEST_PAYLOAD_SIZE - 1; i++)
        payload[i + 1] = COMM_TEST_FRAME_HEADER + 1;

    CommSend(&uart_comm, payload);
}
