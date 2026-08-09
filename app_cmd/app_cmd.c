#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
//
#include "drv_dbus.h"
//
#include "bsp_freertos.h"

// static cmd2gimbal_data_t cmd2gimbal_data;

DBUS_INSTANCE_DEF(dbus_inst);

void AppCmdInit(void)
{
    // 注册 DBUS（仅硬件绑定）
    DBUSRegister(&dbus_inst);

    // 配置 DBUS（硬件映射 + 运行参数）
    DBUS_Config_s dbus_cfg = {
        .uart_e = UART_SBUS,
        .daemon_reload = 100,
        .daemon_fault = DAEMON_FAULT_NONE,
        .lost_timeout_ms = 1000,
    };
    DBUSConfig(&dbus_inst, &dbus_cfg);
}

ITCM_RAM void AppCmdRun(void)
{
    // 设置要发送的数据

    // 通过队列发送出去
    // xQueueOverwrite(cmd2gimbal_queue_handle, &cmd2gimbal_data);
}
