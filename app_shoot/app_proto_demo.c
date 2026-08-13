/**
 * @file app_proto_demo.c
 * @brief APP 自定义协议示例实现（PROTO_DEMO 命令字协议）
 *
 * 帧：  [0xA5] [cmd 1B] [payload N] [CRC16 低字节在前] [0x5A]
 * CRC16 = CRC-16/MODBUS，由 bsp_crc 查表（BSP_CRC_TBL_MODBUS）提供，校验范围 = cmd + payload。
 * 整帧长 = N + 5，全部随 COMM_DEMO_DEF 编译期确定。
 *
 * @note 校验失败（帧头/帧尾/CRC16 不符）的帧直接丢弃；本示例协议不做帧序列检测
 *       （如需 seq/丢帧统计，参考 comm_proto_custom 的实现加状态字段）。
 */

#include "app_proto_demo.h"
#include "app_cfg.h"
//
#include "bsp_crc.h"
#include "bsp_crc_tables.h"
//
#include <string.h>

#ifdef DRV_COMM_USED

static int8_t DemoPack(CommProto *self, const uint8_t *payload, uint8_t *out_buff);
static const uint8_t *DemoUnpack(CommProto *self, const uint8_t *data);
static void DemoReset(CommProto *self);

static const CommProtoVTable_s s_demo_vtable = {
    .pack = DemoPack,
    .unpack = DemoUnpack,
    .reset = DemoReset,
};

/* 打包：帧头 + cmd + payload + CRC16(低字节在前) + 帧尾 */
static int8_t DemoPack(CommProto *self, const uint8_t *payload, uint8_t *out_buff)
{
    CommProtoDemo *p = (CommProtoDemo *)self;
    uint16_t crc;

    if (self == NULL || payload == NULL || out_buff == NULL)
        return -1;

    out_buff[0] = PROTO_DEMO_FRAME_HEADER;
    out_buff[1] = p->cmd;
    memcpy(&out_buff[2], payload, self->payload_size);
    crc = (uint16_t)BSP_CRC_TableCalc(&BSP_CRC_TBL_CRC16_MODBUS, &out_buff[1], self->payload_size + 1);
    out_buff[2 + self->payload_size] = (uint8_t)(crc & 0xFF); /* LSB 在前 */
    out_buff[3 + self->payload_size] = (uint8_t)(crc >> 8);
    out_buff[4 + self->payload_size] = PROTO_DEMO_FRAME_TAIL;
    return 0;
}

/* 只解包：帧头/帧尾/CRC16 校验通过后返回 payload 指针，并把 cmd 读回实例 */
static const uint8_t *DemoUnpack(CommProto *self, const uint8_t *data)
{
    CommProtoDemo *p = (CommProtoDemo *)self;
    uint16_t n = self->payload_size;
    uint16_t crc;

    if (self == NULL || data == NULL)
        return NULL;
    if (data[0] != PROTO_DEMO_FRAME_HEADER)
        return NULL;
    if (data[n + 4] != PROTO_DEMO_FRAME_TAIL)
        return NULL;
    crc = (uint16_t)BSP_CRC_TableCalc(&BSP_CRC_TBL_CRC16_MODBUS, &data[1], n + 1);
    if ((data[n + 2] | (uint16_t)(data[n + 3] << 8)) != crc)
        return NULL;
    p->cmd = data[1];
    return data + 2; /* payload 指针（cmd 之后） */
}

/* 重置打包/解包状态 */
static void DemoReset(CommProto *self)
{
    CommProtoDemo *p = (CommProtoDemo *)self;

    if (self == NULL)
        return;
    p->cmd = 0;
}

int8_t CommProtoDemoInit(CommProtoDemo *proto)
{
    if (proto == NULL || proto->base.media == NULL)
        return -1;
    proto->base.vtable = &s_demo_vtable;
    proto->cmd = 0;
    return 0;
}

/* 后端描述符：app 自定义协议登记（Init 的 void* 封装，类型安全） */
static int8_t BackendDemoInit(void *proto)
{
    return CommProtoDemoInit((CommProtoDemo *)proto);
}

const CommProtoBackend_t g_demo_backend = {
    .type = PROTO_DEMO,
    .init = BackendDemoInit,
};

#endif /* DRV_COMM_USED */
