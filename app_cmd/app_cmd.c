#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
//
#include "drv_sbus.h"
//
#include "bsp_freertos.h"

SBUS_INSTANCE_DEF(sbus_inst);

void AppCmdInit(void)
{
    // 注册 SBUS 实例
    SBUS_Init_Config_s sbus_cfg = {
        .uart_e = UART_SBUS,
        .daemon_reload = 100,
        .daemon_fault = DAEMON_FAULT_NONE,
        .lost_timeout_ms = 1000,
    };
    SBUSRegister(&sbus_inst, &sbus_cfg);
}

ITCM_RAM void AppCmdRun(void)
{
    // 设置要发送的数据
    cmd2chassis_data_t send_data;
    if (sbus_inst.signal_lost)
    {
        send_data.mode = stop;
        send_data.vx = 0.0f;
        send_data.vy = 0.0f;
        send_data.w = 0.0f;
    }
    else
    {
        // 设置速度
        send_data.vx = v_speed * sbus_inst.sbus_data.ch[2];
        send_data.vy = -v_speed * sbus_inst.sbus_data.ch[0];
        send_data.w = -w_speed * sbus_inst.sbus_data.ch[3];
        // 设置模式
        if (sbus_inst.sbus_data.ch[4] < -0.5)
        {
            send_data.mode = stop;
            send_data.vx = 0.0f;
            send_data.vy = 0.0f;
            send_data.w = 0.0f;
        }
        else if (sbus_inst.sbus_data.ch[4] > 0.5)
        {
            send_data.mode = gyro;
        }
        else
        {
            send_data.mode = normal;
        }
    }
    // 通过队列发送出去
    xQueueOverwrite(cmd2chassis_queue_handle, &send_data);
}
