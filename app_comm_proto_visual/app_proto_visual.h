/**
 * @file app_proto_visual.h
 * @brief app 自定义协议-视觉电脑通信协议后端（PROTO_VISUAL）
 *
 * 与原项目（RM_YunQian，origin-trw）视觉线协议一致：固定长度帧，帧 =
 *   [payload N（含 cmd_ID 首字节）] [CRC16 2B]
 * 无帧头/帧尾/seq（帧定界靠 cmd_ID + 固定长度 + CRC16）。仅协议编解码：
 *   - payload = 原协议帧去掉尾 2B CRC16 的完整字节（含 cmd_ID）
 *       * 接收 payload = 48B（原 50B 帧：cmd_ID + 数据 + CRC16）
 *       * 发送 payload = 55B（原 57B 帧）
 *   - CRC16 = 裁判系统官方算法（实际为 CRC-16/MCRF4XX）：poly 0x1021、LSB-first
 *     反射（refin/refout=1）、init 0xFFFF、xor_out 0、附加低字节在前。与 bsp_crc
 *     现有表（CCITT-FALSE/KERMIT/MODBUS）均不匹配，实现用 BSP_CRC_Direct 逐位计算
 *     （零 RAM 表，1kHz 57B 帧开销可忽略）。
 * 数据字段结构体不内嵌：payload 即帧体完整字节，业务层按帧偏移自行解析
 * （接收 data[0]==VISUAL_CMD_RX，时间戳/各 float 偏移见需求文档）。
 *
 * 配套 media 后端：usb_simple（整帧 ≤64B 免序号单包透传），50B/57B 帧原样一包进出，
 * 线协议完全兼容视觉电脑（VCP 虚拟串口）。
 *
 * @note COMM_DEF 通过 token 拼接 COMM_##proto_type_##_DEF 分发到本宏；
 *       协议 id（PROTO_VISUAL）与开销宏（PROTO_VISUAL_OVERHEAD）供 COMM_DEF 拼接。
 * @note 收发 payload 可不等长（rx 48B / tx 55B）：media 缓冲自动 = 48+2=50 / 55+2=57，
 *       对齐原协议帧长。协议层按 self->payload_size 编解码，天然支持。
 */

#ifndef __APP_PROTO_VISUAL_H
#define __APP_PROTO_VISUAL_H

#include "comm_proto.h"

#ifdef DRV_COMM_USED

/* app 自定义协议 id（PROTO_USER 起分配；与 app_proto_demo.h 的 PROTO_DEMO 并列） */
#define PROTO_VISUAL (PROTO_USER + 1)

/* 协议开销 = 帧尾 CRC16（2B）；整帧长 = payload_size + PROTO_VISUAL_OVERHEAD */
#define PROTO_VISUAL_OVERHEAD 2

/* 视觉线协议 cmd_ID 常量见 app.h 的 vision_cmd_e 枚举：
 *   接收帧（视觉→板）首字节 = VISUAL_CMD_RX，发送帧（板→视觉）首字节 = VISUAL_CMD_TX */

/* 视觉协议派生结构体：仅编解码，不内嵌数据字段（首成员必须为 CommProto 基类） */
typedef struct
{
    CommProto base; /* 基类（首成员） */
} CommProtoVisual;

/**
 * @brief 静态定义视觉协议实例
 * @param name        实例名称
 * @param media_      media 实例（发送用，指向 CommMedia 派生实例）
 * @param payload_sz  payload 长度（编译期确定；整帧长 = payload_sz + PROTO_VISUAL_OVERHEAD）
 *
 * @note media_ 以指针绑定，运行时无需另传；发送缓冲由 comm 层提供（inst->tx_buff，
 *       大小由 COMM_DEF 按开销推算）。vtable 由 CommProtoVisualInit 挂接。
 *
 * @example
 *   COMM_PROTO_VISUAL_DEF(proto_vis, vis_comm_media, 55);  // 发送 payload 55B
 */
#define COMM_PROTO_VISUAL_DEF(name, media_, payload_sz) \
    static CommProtoVisual name = {                     \
        .base.payload_size = payload_sz,                \
        .base.media = (void *)&media_} /* 尾部无分号，调用处加 */

/**
 * @brief 初始化视觉协议后端（挂 vtable；无内部状态）
 * @param proto CommProtoVisual 实例指针（COMM_PROTO_VISUAL_DEF 定义）
 * @retval 0 成功；-1 参数非法
 */
int8_t CommProtoVisualInit(CommProtoVisual *proto);

/* 后端描述符：供 app 启动时 CommProtoRegisterBackend 登记（须在 CommRegister 之前） */
extern const CommProtoBackend_t g_visual_backend;

#endif /* DRV_COMM_USED */
#endif /* __APP_PROTO_VISUAL_H */
