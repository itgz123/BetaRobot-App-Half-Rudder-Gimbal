/**
 * @file app_proto_demo.h
 * @brief APP 自定义协议示例：命令字协议（PROTO_DEMO，app 层完全自包含）
 *
 * 演示「comm 驱动支持 app 自定义协议」的完整契约（驱动零改动）：
 *   - 类型 id：PROTO_DEMO = PROTO_USER + 1（app 自定义空间，>= PROTO_USER）
 *   - 开销宏：PROTO_DEMO_OVERHEAD（COMM_DEF 按 token 拼接 PROTO_DEMO_OVERHEAD 定缓冲）
 *   - DEF 宏：COMM_PROTO_DEMO_DEF（COMM_DEF 按 token 拼接 COMM_PROTO_##token##_DEF 分发）
 *   - 派生结构体 + vtable + Init + 后端描述符 g_demo_backend
 *
 * 帧格式（N = payload_size，固定长度）：
 *   [帧头 0xA5] [cmd 1B] [payload N] [CRC16 2B] [帧尾 0x5A]
 *   CRC16 = CRC-16/MODBUS（bsp_crc 查表 BSP_CRC_TBL_CRC16_MODBUS），校验范围 = cmd + payload，
 *   帧内低字节在前（MODBUS 惯例）。整帧长 = N + PROTO_DEMO_OVERHEAD(5)。
 *
 * 使用（见 app_shoot.c）：
 *   CommProtoRegisterBackend(&g_demo_backend);                  // 1. 登记自定义协议
 *   COMM_DEF(demo_comm, MEDIA_USART, DEMO, DEMO, 8, 8, ...);    // 2. 接线（token = DEMO）
 *   CommRegister(&demo_comm); CommConfig(&demo_comm, &cfg);     // 3. 注册 + 配置
 *   ((CommProtoDemo *)demo_comm.tx_proto)->cmd = 0x01;          // 4. 发送前设命令字
 *   CommSend(&demo_comm, payload);
 */

#ifndef __APP_PROTO_DEMO_H
#define __APP_PROTO_DEMO_H

#include "comm_proto.h"

#ifdef DRV_COMM_USED

/* 协议 id（>= PROTO_USER 为 app 自定义空间）与开销（COMM_DEF 按 PROTO_DEMO token 拼接） */
#define PROTO_DEMO (PROTO_USER + 1)
#define PROTO_DEMO_OVERHEAD 5 /* 帧头(1) + cmd(1) + CRC16(2) + 帧尾(1) */

/* 帧头/帧尾定界（CRC16 校验范围 = cmd + payload，见 app_proto_demo.c） */
#define PROTO_DEMO_FRAME_HEADER 0xA5
#define PROTO_DEMO_FRAME_TAIL 0x5A

/* 命令字协议派生结构体（首成员必须为 CommProto 基类） */
typedef struct
{
    CommProto base; /* 基类（首成员） */
    uint8_t cmd;    /* 命令字：pack 时写帧；unpack 时从帧读回（业务层按此分发） */
} CommProtoDemo;

/**
 * @brief 静态定义命令字协议实例
 * @param name        实例名称
 * @param media_      media 实例（发送用，指向 CommMedia 派生实例）
 * @param payload_sz  payload 长度（编译期确定；整帧长 = payload_sz + PROTO_DEMO_OVERHEAD）
 *
 * @example
 *   COMM_PROTO_DEMO_DEF(demo_rx_proto, demo_media, 8);
 */
#define COMM_PROTO_DEMO_DEF(name, media_, payload_sz) \
    static CommProtoDemo name = {                     \
        .base.payload_size = payload_sz,              \
        .base.media = (void *)&media_,                \
        .cmd = 0} /* 尾部无分号，调用处加 */

/**
 * @brief 初始化命令字协议后端（挂 vtable + 清状态）
 * @param proto CommProtoDemo 实例指针（COMM_DEMO_DEF 定义）
 * @retval 0 成功；-1 参数非法
 */
int8_t CommProtoDemoInit(CommProtoDemo *proto);

/* 后端描述符：启动时 CommProtoRegisterBackend(&g_demo_backend) 登记自定义协议 */
extern const CommProtoBackend_t g_demo_backend;

#endif /* DRV_COMM_USED */
#endif /* __APP_PROTO_DEMO_H */
