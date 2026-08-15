#include "app_usb.h"
#include "app_cfg.h"
#include "bsp_usb.h"

/* USB 实例：RX 环 256B（可用 255B），TX op 环 4（可用 3 个挂起 op）
 * 配置 1 = CDC + HID 键盘（复合设备）；配置 2 = 仅 HID 键盘（单类） */
USB_INSTANCE_DEF(usb_vcp, 256, 4);

/* 回环缓冲 */
static uint8_t s_loop_buf[256];

/* TODO[HID停用]：Windows 复合设备 HID 接口枚举异常（配置描述符正确但 Windows 不实例化接口 2），
 * 暂未解决。HID 类已从 bsp/CMakeLists.txt 移除编译；恢复时：取消本文件各 #if 0 块、
 * 解开 bsp_usb.h 中 hid_kbd 字段与 #include，并把 bsp_usb/hid/bsp_usb_hid_kbd.c 加回 CMake。 */
#if 0
volatile int32_t usb_hid_send_ret = 0; /* 调试：USBHIDKbdSend 返回值（volatile 防优化，调试器 watch） */
#endif

void AppUsbInit(void)
{
    USB_Config_s cfg = {
        .usb_e = USB_MAIN, /* 每块板 bsp_map.h 映射到自己的 USB 外设 */
        .vid = 0x1234,
        .pid = 0x5678,
        .bcd = 0x0100,
    };

    (void)USBRegister(&usb_vcp);
    (void)USBConfig(&usb_vcp, &cfg); /* 默认挂 CDC 到配置 1 */

    /* TODO[HID停用]：HID 键盘注册暂停（见文件顶部注释），恢复时取消 #if 0 */
#if 0
    /* 配置 1 = CDC + HID 键盘（SET_CONFIGURATION(1)，默认） */
    (void)USBAddClass(&usb_vcp, 0, USB_HIDKbdVTable(), &usb_vcp.hid_kbd);
    /* 配置 2 = 仅 HID 键盘（SET_CONFIGURATION(2) 切换；单类无 IAD → 设备描述符类字段自动归零） */
    (void)USBAddClass(&usb_vcp, 1, USB_HIDKbdVTable(), &usb_vcp.hid_kbd);
#endif

    (void)USBStart(&usb_vcp);
}

/* 回环：PC 发来的数据原样转发回去（验证 RX 背压 + TX op + ZLP） */
void AppUsbRun(void)
{
    int32_t n = USBReceive(&usb_vcp, s_loop_buf, sizeof(s_loop_buf));
    if (n > 0)
    {
        (void)USBTransmit(&usb_vcp, s_loop_buf, (uint16_t)n);
    }

    /* TODO[HID停用]：HID 键盘演示暂停（见文件顶部注释），恢复时取消 #if 0 */
#if 0
    /* HID 键盘演示：周期交替发送 'A' 按下/释放 report（验证 HID report 数据通路） */
    static uint32_t s_cnt = 0;
    if ((s_cnt++ & 0x1F) == 0) /* 约每 32 拍一拍 */
    {
        uint8_t report[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        report[2] = (s_cnt & 0x20) ? 0x04 : 0x00; /* 0x04 = Usage 'A' */
        usb_hid_send_ret = USBHIDKbdSend(&usb_vcp, report);
    }
#endif
}
