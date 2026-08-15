#ifndef __APP_USB_H
#define __APP_USB_H

/**
 * @brief USB CDC 虚拟串口初始化（USBRegister/USBConfig/USBStart）
 * @note 在 function_in_main_c 中断关闭区内调用
 */
void AppUsbInit(void);

/**
 * @brief USB 回环任务体：USBReceive -> USBTransmit（验证收发通路）
 * @note 由 app.c 的任务注册循环调用
 */
void AppUsbRun(void);

#endif // !__APP_USB_H
