/**
 * @file app_proto_visual.c
 * @brief 视觉电脑通信协议后端（PROTO_VISUAL）实现
 *
 * 帧格式（固定长度，帧 = payload + CRC16；media 层按固定帧长保证整帧，无需命令字定界）：
 *   - 发送（板→视觉）：55B payload（首字节 cmd_ID=0x02）→ 帧 57B
 *   - 接收（视觉→板）：帧 50B → CRC16 校验 → payload 48B
 *
 * 编解码：
 *   - VisualPack：帧体 = payload（含 cmd_ID）原样拷贝 → 尾部追加 CRC16（低字节在前）
 *   - VisualUnpack：CRC16 校验；通过返回 data（payload 含 cmd_ID，业务层按偏移解析），失败返回 NULL
 *   - VisualReset：无内部状态（固定长度 + 每帧独立校验），空操作
 *
 * CRC16 = 裁判系统官方算法（crc_ref.c，实际为 CRC-16/MCRF4XX）：poly 0x1021、
 * LSB-first 反射（refin/refout=1）、init 0xFFFF、xor_out 0、低字节在前附加。
 * 与 bsp_crc 内置表（CCITT-FALSE/KERMIT/MODBUS 等）均不匹配，用 BSP_CRC_Direct
 * 以算法参数逐位计算，无需建表（零 RAM；1kHz 57B 帧开销可忽略）。
 * @note crc_ref 的 wCRC_Table 经 Python 逐项验证 = poly 0x1021 反射表（256/256 匹配），
 *       非资料中常误写的 poly 0x8005 MSB-first；算法参数据此确定。
 */

#include "app_proto_visual.h"
#include "bsp_crc.h"
#include <string.h>

#ifdef DRV_COMM_USED

/* 裁判系统 CRC16 算法参数 = CRC-16/MCRF4XX：{init 0xFFFF, 16, poly 0x1021, xor_out 0,
 * refin 1, refout 1}。BSP_CRC_Direct 反射分支（LSB-first，内部取 poly 反射 0x8408）
 * + refin==refout 不反转结果，与 crc_ref 查表算法一致（Python 逐项验证）。 */
static const BSP_CRC_Algo_t s_visual_crc_algo = {
    .init_value = 0xFFFF,
    .poly_size = 16,
    .poly = 0x1021,
    .xor_out = 0x0000,
    .reverse_in = 1,
    .reverse_out = 1,
};

static int8_t VisualPack(CommProto *self, const uint8_t *payload, uint8_t *out_buff);
static const uint8_t *VisualUnpack(CommProto *self, const uint8_t *data);
static void VisualReset(CommProto *self);

static const CommProtoVTable_s s_visual_vtable = {
    .pack = VisualPack,
    .unpack = VisualUnpack,
    .reset = VisualReset,
};

/* 打包：帧体 = payload（含 cmd_ID）原样拷贝 → 尾部追加 CRC16（低字节在前）。
 * 1) out_buff = comm 打包缓冲（inst->tx_buff，大小 = payload_size + 2）
 * 2) CRC 校验范围 = 帧体（payload_size 字节，含 cmd_ID），与 crc_ref 数据区一致 */
static int8_t VisualPack(CommProto *self, const uint8_t *payload, uint8_t *out_buff)
{
    CommProtoVisual *p = (CommProtoVisual *)self;
    uint32_t crc;

    if (self == NULL || payload == NULL || out_buff == NULL)
        return -1;
    (void)p;

    memcpy(out_buff, payload, self->payload_size); /* 帧体 = payload（含 cmd_ID） */
    crc = BSP_CRC_Direct(&s_visual_crc_algo, out_buff, self->payload_size);
    out_buff[self->payload_size] = (uint8_t)(crc & 0xFF); /* CRC16 低字节在前 */
    out_buff[self->payload_size + 1] = (uint8_t)((crc >> 8) & 0xFF);
    return 0;
}

/* 解包：仅 CRC16 校验（校验范围 = data[0..payload_size)）。
 * 帧边界由 media 层固定帧长保证，无需 cmd_ID 定界（cmd_ID 随帧传递，仅作业务字段）。
 * 通过返回 data（payload 含 cmd_ID），失败返回 NULL。 */
static const uint8_t *VisualUnpack(CommProto *self, const uint8_t *data)
{
    uint32_t crc_calc, crc_recv;

    if (self == NULL || data == NULL)
        return NULL;

    crc_calc = BSP_CRC_Direct(&s_visual_crc_algo, data, self->payload_size);
    crc_recv = (uint32_t)data[self->payload_size] |
               ((uint32_t)data[self->payload_size + 1] << 8);
    if (crc_calc != crc_recv)
        return NULL; /* CRC 不符：丢弃 */

    return data; /* payload = data（含 cmd_ID），长度 = payload_size（media 层已校验帧长） */
}

/* 重置解包状态：固定长度 + 每帧独立校验，无内部状态 */
static void VisualReset(CommProto *self)
{
    (void)self;
}

int8_t CommProtoVisualInit(CommProtoVisual *proto)
{
    if (proto == NULL)
        return -1;
    proto->base.vtable = &s_visual_vtable;
    proto->base.on_frame = NULL; /* 出帧回调由 CommConfig 挂接 */
    return 0;
}

/* 后端初始化回调（void* 封装，注册表分发用） */
static int8_t BackendVisualInit(void *proto)
{
    return CommProtoVisualInit((CommProtoVisual *)proto);
}

/* 后端描述符：.type = PROTO_VISUAL，app 启动时 CommProtoRegisterBackend(&g_visual_backend) 登记 */
const CommProtoBackend_t g_visual_backend = {
    .type = PROTO_VISUAL,
    .init = BackendVisualInit,
};

#endif /* DRV_COMM_USED */
