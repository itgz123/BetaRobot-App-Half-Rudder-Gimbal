#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
//
#include "drv_dbus.h"
//
#include "bsp_freertos.h"
#include "bsp_assert.h"

typedef enum
{
    no_control_e = 0,
    dbus_e = 1,
    photo_story_e = 2,
    keyboard_mouse_e = 3,
    visual_control_e = 4,
} cmd_control_type; // 控制类型

static cmd2gimbal_data_t cmd_cmd2gimbal_data;
static cmd_control_type used_remote_control;

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
        used_remote_control = dbus_e;
    }

    // 2. 设置要发送的数据
    cmd_cmd2gimbal_data.state = disable; // 先失能

    if (dbus_e == used_remote_control) // 使用dbus遥控
    {
        if (DBUS_SW_MID == dbus_inst.dbus_data.s1)
        {
            cmd_cmd2gimbal_data.state = enable;
        }
    }
    else if (photo_story_e == used_remote_control) // 使用图传遥控
    {
    }
    else if (keyboard_mouse_e == used_remote_control) // 使用图传键鼠
    {
    }
    else if (visual_control_e == used_remote_control) // 使用视觉
    {
    }
    else // 全部失效
    {
        cmd_cmd2gimbal_data.state = disable;
    }

    // 3. 通过队列发送出去
    xQueueOverwrite(cmd2gimbal_queue_handle, &cmd_cmd2gimbal_data);
}
