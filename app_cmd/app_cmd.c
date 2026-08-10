#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
//
#include "drv_dbus.h"
//
#include "bsp_freertos.h"
#include "bsp_assert.h"

static cmd2gimbal_data_t cmd_cmd2gimbal_data;
static uint8_t used_remote_control; // 1：dbus，2；图传遥控，3：键鼠

DBUS_INSTANCE_DEF(dbus_inst);

void AppCmdInit(void)
{
    // 注册 DBUS（仅硬件绑定）
    BSP_ASSERT_APP_CALL(DBUSRegister(&dbus_inst));

    // 配置 DBUS（硬件映射 + 运行参数）
    DBUS_Config_s dbus_cfg = {
        .uart_e = UART_SBUS,
        .daemon_reload = 100,
        .daemon_fault = DAEMON_FAULT_NONE,
        .lost_timeout_ms = 1000,
    };
    BSP_ASSERT_APP_CALL(DBUSConfig(&dbus_inst, &dbus_cfg));
}

ITCM_RAM void AppCmdRun(void)
{
    // 1. 根据优先级和连接情况判断使用dbus遥控还是图传还是键鼠
    // 现在只有dbus
    if ((dbus_inst.daemon->is_online == 1) || (dbus_inst.dbus_data.failsafe = 0))
    {
        used_remote_control = 1;
    }

    // 2. 设置要发送的数据
    cmd_cmd2gimbal_data.state = disable; // 先失能

    if (1 == used_remote_control) // 使用dbus遥控
    {
        if (DBUS_SW_UP == dbus_inst.dbus_data.s1)
        {
            cmd_cmd2gimbal_data.state = enable;
        }
    }
    else if (2 == used_remote_control)
    {
    }
    else if (3 == used_remote_control)
    {
    }
    else
    {
        cmd_cmd2gimbal_data.state = disable;
    }

    // 3. 通过队列发送出去
    xQueueOverwrite(cmd2gimbal_queue_handle, &cmd_cmd2gimbal_data);
}
