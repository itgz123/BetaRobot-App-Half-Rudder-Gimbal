/**
 * @file app_shoot.c
 * @brief 发射机构 App（当前为通信框架 UART 测试）
 *
 * 通信链路测试（comm 框架 media + 协议 + 顶层分发），两条对话：
 *   - uart_comm（内置 PROTO_CUSTOM）：UART_10, USART10, 921600，帧 = A5 [seq] [payload] [crc8] 5A
 *   - demo_comm（app 自定义 PROTO_DEMO）：UART_7 图传口，帧 = A5 [cmd] [payload] [crc16] 5A
 *     演示「comm 驱动支持 app 自定义协议」：协议后端完全在 app 层（app_proto_demo.c），
 *     启动时 CommProtoRegisterBackend 登记，COMM_DEF 传协议名 token（DEMO）接线，驱动零改动。
 *
 * 两段式接口：CommRegister（不可重入，注册 + 接线分发）
 *           → CommConfig（可重入，介质参数 + 出帧回调，均可反复修改）
 * 发送：AppShootRun 每 100ms 各发一帧（CommSend，走各发送协议）
 *
 * @note 测试方式（硬件回环）：把 UART_10 的 TX/RX 短接，串口助手可看到
 *       板子周期发帧（帧格式 A5 [seq] [payload] [crc] 5A）；助手回发的数据
 *       也会被识别（帧头/帧尾/CRC8/seq 校验）并分发到 CommTestOnFrame。
 *       demo_comm 同理走 UART_7（CRC16 校验范围 = cmd + payload）。
 *       USB 接收测试：电脑发特定 100B 帧（USB 传输层拆成 2 个分包包），板子
 *       media 层按分包序号重组成完整帧后 UsbTestOnFrame 触发，全局 usb_rx_count 加一。
 */

#include "app_shoot.h"
#include "app_cfg.h"
//
#include "bsp_uart_log.h"
#include "bsp_usart.h"
//
#include "drv_comm.h"
#include "comm_media_usart.h"
#include "comm_media_usb.h"
#include "comm_proto_raw.h"
#include "comm_proto_custom.h"
#include "app_proto_demo.h"

uint16_t test = 0;
uint16_t usb_rx_count = 0; /* USB 接收测试计数：电脑发特定 100B 帧，收满一帧 +1 */

#define COMM_TEST_RX_SIZE 8 /* 接收 payload 大小（PROTO_CUSTOM，帧长 = payload + 4） */
#define COMM_TEST_TX_SIZE 8 /* 发送 payload 大小 */
#define DEMO_RX_SIZE 8      /* demo 自定义协议（PROTO_DEMO）收发 payload 大小 */
#define DEMO_TX_SIZE 8
#define USB_COMM_SIZE 100 /* USB(CDC) 对话收发 payload 大小（RAW；>63B 触发 media 层分包发送） */

/* 内置自定义帧协议对话：接收/发送 PROTO_CUSTOM（8B），UART_10，ISR 直解 */
COMM_DEF(uart_comm, MEDIA_USART, CUSTOM, CUSTOM, COMM_TEST_RX_SIZE, COMM_TEST_TX_SIZE, UNPACK_IN_ISR);
/* app 自定义协议对话：接收/发送 PROTO_DEMO（8B），UART_7，ISR 直解 */
COMM_DEF(demo_comm, MEDIA_USART, DEMO, DEMO, DEMO_RX_SIZE, DEMO_TX_SIZE, UNPACK_IN_ISR);
/* USB(CDC) 对话：接收/发送 PROTO_RAW（100B），ISR 直解。
 * 100B > 63B 触发 media 层分包：每包 = [分包序号][数据片 ≤ 63B]，序号由 media 层维护 */
COMM_DEF(usb_comm, MEDIA_USB, RAW, RAW, USB_COMM_SIZE, USB_COMM_SIZE, UNPACK_IN_ISR);

static void CommTestOnFrame(const uint8_t *payload)
{
    /* 打印收到的帧序列（回环下 = 本地发送 seq，连续递增验证 seq/丢帧检测） */
    LOGINFO("[app_shoot] rx seq=%u", (unsigned)payload[0]);
    test++;
}

/* demo 自定义协议出帧回调：命令字由协议层读回实例（payload[0] 为业务数据首字节） */
static void DemoOnFrame(const uint8_t *payload)
{
    uint8_t cmd = ((CommProtoDemo *)demo_comm.rx_proto)->cmd;

    (void)cmd; /* UART_LOG_USED 未启用时 LOGINFO 为空宏，避免 cmd 未使用告警 */
    LOGINFO("[app_shoot] demo rx cmd=0x%02X payload0=%u", (unsigned)cmd, (unsigned)payload[0]);
}

/* USB 对话出帧回调：RAW 协议直接收重组后的完整协议帧（分包序号已由 media 层剥掉，
 * 丢帧计数 lost_frames 保留在 usb_comm_media 实例，可查询） */
static void UsbTestOnFrame(const uint8_t *payload)
{
    /* USB 接收测试：电脑发送特定 100B 帧（>63B，USB 传输层拆成 [0][63B][1][37B]
     * 两个分包包），media 层按分包序号重组成完整帧后进本回调，全局计数 +1。
     * 分包错位/丢包会被丢帧重同步，只有完整帧才回调，不会误计数。
     * payload[0] 打印用于核对收到的正是电脑发送的特定内容（如首字节 0xAA）。 */
    usb_rx_count++;
    LOGINFO("[app_shoot] usb rx payload0=%u cnt=%u", (unsigned)payload[0], (unsigned)usb_rx_count);
}

void AppShootInit(void)
{
    /* 内置协议对话（CUSTOM，UART_10）：介质参数 + 出帧回调（可重入，运行期可改） */
    CommConfig_s cfg = {
        .media_cfg = &(USART_Config_s){
            .uart_e = UART_10,
            .tx_mode = USART_DMA_MODE,
        },
        .on_frame = CommTestOnFrame,
    };
    CommRegister(&uart_comm); /* 两段式：注册（不可重入）→ 配置（可重入） */
    CommConfig(&uart_comm, &cfg);

    /* app 自定义协议对话（DEMO，UART_7 图传口）：登记后端必须在 CommRegister 之前 */
    CommProtoRegisterBackend(&g_demo_backend);
    CommConfig_s demo_cfg = {
        .media_cfg = &(USART_Config_s){
            .uart_e = UART_7,
            .tx_mode = USART_DMA_MODE,
        },
        .on_frame = DemoOnFrame,
    };
    CommRegister(&demo_comm);
    CommConfig(&demo_comm, &demo_cfg);

    /* USB(CDC) 对话：media_cfg 传空配置（USB 无运行期参数，接收钩子由 media 层强制接管）。
     * @note 须在 MX_USB_DEVICE_Init 之后注册/发送（见 function_in_main_c 改动） */
    CommConfig_s usb_cfg = {
        .media_cfg = &(USB_Config_s){0},
        .on_frame = UsbTestOnFrame,
    };
    CommRegister(&usb_comm);
    CommConfig(&usb_comm, &usb_cfg);
}

void AppShootRun(void)
{
    uint8_t payload[COMM_TEST_TX_SIZE];
    uint8_t usb_payload[USB_COMM_SIZE];

    /* 构造递增 payload（协议层负责加帧头/seq/CRC8/帧尾；payload 首字节可当发送 seq 观察） */
    for (int i = 0; i < COMM_TEST_TX_SIZE; i++)
        payload[i] = (uint8_t)i;

    CommSend(&uart_comm, payload);

    /* demo 自定义协议：发送前设置命令字（示例固定 0x01，业务可按需改） */
    ((CommProtoDemo *)demo_comm.tx_proto)->cmd = 0x01;
    CommSend(&demo_comm, payload);

    /* USB 对话：RAW 100B payload（>63B，media 层自动分包 [0][63B] [1][37B] 发出） */
    for (int i = 0; i < USB_COMM_SIZE; i++)
        usb_payload[i] = (uint8_t)i;
    CommSend(&usb_comm, usb_payload);
}
