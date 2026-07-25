#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
//
#include "drv_sbus.h"
//
#include "bsp_freertos.h"

// static cmd2gimbal_data_t cmd2gimbal_data;

SBUS_INSTANCE_DEF(sbus_inst);

void AppCmdInit(void)
{
    // 注册 SBUS（仅硬件绑定）
    SBUSRegister(&sbus_inst, UART_SBUS);

    // 配置 SBUS（运行参数）
    SBUS_Config_s sbus_cfg = {
        .daemon_reload = 100,
        .daemon_fault = DAEMON_FAULT_NONE,
    };
    SBUSConfig(&sbus_inst, &sbus_cfg);
}

ITCM_RAM void AppCmdRun(void)
{
    // 设置要发送的数据

    // 通过队列发送出去
    // xQueueOverwrite(cmd2gimbal_queue_handle, &cmd2gimbal_data);
}
