#include "app_chassis.h"
#include "app_cfg.h"
#include "app.h"
//
#include "drv_chassis_position_lite.h"
#include "drv_djimotor.h"
#include "drv_motor_base.h"
#include "drv_daemon.h"
#include "drv_vofa.h"
//
#include "bsp_math.h"
#include "bsp_freertos.h"
#include "bsp_dwt.h"

DJIMOTOR_INSTANCE_DEF(motor_lf1);
DJIMOTOR_INSTANCE_DEF(motor_lb2);
DJIMOTOR_INSTANCE_DEF(motor_rb3);
DJIMOTOR_INSTANCE_DEF(motor_rf4);
CHASSIS_POSITION_LITE_INSTANCE_DEF(chassis);

void AppChassisInit(void)
{
    DJIMotor_Init_Config_s config_motor_lf1 = {
        .can_e = CAN_2,
        .controller_setting = {
            .angle_external_ptr = NULL,
            .angle_limit_max = 0.0f,
            .angle_limit_min = 0.0f,
            .angle_src = MOTOR_FEEDBACK_MOTOR,
            .feedback_direction = MOTOR_DIRECTION_NORMAL,
            .loop_type = MOTOR_LOOP_SPEED,
            .motor_direction = MOTOR_DIRECTION_NORMAL,
            .position_feedforward_ptr = NULL,
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .position_mode = MOTOR_POSITION_CONTINUOUS,
            .speed_external_ptr = NULL,
            .speed_feedforward_ptr = NULL,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .speed_src = MOTOR_FEEDBACK_MOTOR,
        },
        .fault_action = DAEMON_FAULT_NONE,
        .model = DJI_MODEL_M3508,
        .motor_id = 1,
        .pid_angle_setting = {},
        .pid_speed_setting = {
            .kp = 0.08f,
            .ki = 0.001f,
            .kd = 0.012f,
            .integral_limit = 20.0f, // 积分限幅阈值 (0 = 禁用)
            .coef_a = 80.0f,         // 变速积分参数 A (0 = 禁用)
            .coef_b = 50.0f,         // 变速积分参数 B
            .d_lpf_rc = 0.0f,        // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0.0f,      // 输出滤波时间常数 RC (0 = 禁用)

            .config_mask = PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_DERIVATIVE_ON_MEAS | PID_ENABLE_CHANGING_INTEGRATION, // 功能配置掩码
        },
        .reload_count = 10,
        .speed_lpf_enable = ENABLE,
        .speed_lpf_rc = 0.1,
    };
    DJIMotorRegister(&motor_lf1, &config_motor_lf1);
    DJIMotor_Init_Config_s config_motor_lb2 = {
        .can_e = CAN_2,
        .controller_setting = {
            .angle_external_ptr = NULL,
            .angle_limit_max = 0.0f,
            .angle_limit_min = 0.0f,
            .angle_src = MOTOR_FEEDBACK_MOTOR,
            .feedback_direction = MOTOR_DIRECTION_NORMAL,
            .loop_type = MOTOR_LOOP_SPEED,
            .motor_direction = MOTOR_DIRECTION_NORMAL,
            .position_feedforward_ptr = NULL,
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .position_mode = MOTOR_POSITION_CONTINUOUS,
            .speed_external_ptr = NULL,
            .speed_feedforward_ptr = NULL,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .speed_src = MOTOR_FEEDBACK_MOTOR,
        },
        .fault_action = DAEMON_FAULT_NONE,
        .model = DJI_MODEL_M3508,
        .motor_id = 2,
        .pid_angle_setting = {},
        .pid_speed_setting = {
            .kp = 0.08f,
            .ki = 0.001f,
            .kd = 0.012f,
            .integral_limit = 20.0f, // 积分限幅阈值 (0 = 禁用)
            .coef_a = 80.0f,         // 变速积分参数 A (0 = 禁用)
            .coef_b = 50.0f,         // 变速积分参数 B
            .d_lpf_rc = 0.0f,        // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0.0f,      // 输出滤波时间常数 RC (0 = 禁用)

            .config_mask = PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_DERIVATIVE_ON_MEAS | PID_ENABLE_CHANGING_INTEGRATION, // 功能配置掩码
        },
        .reload_count = 10,
        .speed_lpf_enable = ENABLE,
        .speed_lpf_rc = 0.1,
    };
    DJIMotorRegister(&motor_lb2, &config_motor_lb2);
    DJIMotor_Init_Config_s config_motor_rb3 = {
        .can_e = CAN_2,
        .controller_setting = {
            .angle_external_ptr = NULL,
            .angle_limit_max = 0.0f,
            .angle_limit_min = 0.0f,
            .angle_src = MOTOR_FEEDBACK_MOTOR,
            .feedback_direction = MOTOR_DIRECTION_NORMAL,
            .loop_type = MOTOR_LOOP_SPEED,
            .motor_direction = MOTOR_DIRECTION_NORMAL,
            .position_feedforward_ptr = NULL,
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .position_mode = MOTOR_POSITION_CONTINUOUS,
            .speed_external_ptr = NULL,
            .speed_feedforward_ptr = NULL,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .speed_src = MOTOR_FEEDBACK_MOTOR,
        },
        .fault_action = DAEMON_FAULT_NONE,
        .model = DJI_MODEL_M3508,
        .motor_id = 3,
        .pid_angle_setting = {},
        .pid_speed_setting = {
            .kp = 0.08f,
            .ki = 0.001f,
            .kd = 0.012f,
            .integral_limit = 20.0f, // 积分限幅阈值 (0 = 禁用)
            .coef_a = 80.0f,         // 变速积分参数 A (0 = 禁用)
            .coef_b = 50.0f,         // 变速积分参数 B
            .d_lpf_rc = 0.0f,        // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0.0f,      // 输出滤波时间常数 RC (0 = 禁用)

            .config_mask = PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_DERIVATIVE_ON_MEAS | PID_ENABLE_CHANGING_INTEGRATION, // 功能配置掩码
        },
        .reload_count = 10,
        .speed_lpf_enable = ENABLE,
        .speed_lpf_rc = 0.1,
    };
    DJIMotorRegister(&motor_rb3, &config_motor_rb3);
    DJIMotor_Init_Config_s config_motor_rf4 = {
        .can_e = CAN_2,
        .controller_setting = {
            .angle_external_ptr = NULL,
            .angle_limit_max = 0.0f,
            .angle_limit_min = 0.0f,
            .angle_src = MOTOR_FEEDBACK_MOTOR,
            .feedback_direction = MOTOR_DIRECTION_NORMAL,
            .loop_type = MOTOR_LOOP_SPEED,
            .motor_direction = MOTOR_DIRECTION_NORMAL,
            .position_feedforward_ptr = NULL,
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .position_mode = MOTOR_POSITION_CONTINUOUS,
            .speed_external_ptr = NULL,
            .speed_feedforward_ptr = NULL,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,
            .speed_src = MOTOR_FEEDBACK_MOTOR,
        },
        .fault_action = DAEMON_FAULT_NONE,
        .model = DJI_MODEL_M3508,
        .motor_id = 4,
        .pid_angle_setting = {},
        .pid_speed_setting = {
            .kp = 0.08f,
            .ki = 0.001f,
            .kd = 0.012f,
            .integral_limit = 20.0f, // 积分限幅阈值 (0 = 禁用)
            .coef_a = 80.0f,         // 变速积分参数 A (0 = 禁用)
            .coef_b = 50.0f,         // 变速积分参数 B
            .d_lpf_rc = 0.0f,        // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0.0f,      // 输出滤波时间常数 RC (0 = 禁用)

            .config_mask = PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_DERIVATIVE_ON_MEAS | PID_ENABLE_CHANGING_INTEGRATION, // 功能配置掩码
        },
        .reload_count = 10,
        .speed_lpf_enable = ENABLE,
        .speed_lpf_rc = 0.1,
    };
    DJIMotorRegister(&motor_rf4, &config_motor_rf4);
    // 注册底盘 (OMNI_X, 参数全0)
    ChassisPositionLite_Cfg_s cfg_chassis = {
        .wheel_radius = 0.075f,
        .reduction_ratio = 20.0f,
        .x = 0.32f,
        .y = 0.32f,
        .chassis_type = CHASSISTYPE_OMNI_X,
        .motor = {&motor_lb2.base, &motor_lf1.base, &motor_rf4.base, &motor_rb3.base},
    };
    ChassisPositionLite_Init(&chassis, &cfg_chassis);
    // 设置偏置为0

    MotorSetOffset(&motor_lf1.base, 0.0f);
    MotorSetOffset(&motor_lb2.base, 0.0f);
    MotorSetOffset(&motor_rb3.base, 0.0f);
    MotorSetOffset(&motor_rf4.base, 0.0f);
    // 使能
    MotorEnable(&motor_lf1.base);
    MotorEnable(&motor_lb2.base);
    MotorEnable(&motor_rb3.base);
    MotorEnable(&motor_rf4.base);
}

ITCM_RAM void AppChassisRun(void)
{
    // 从队列获取cmd消息
    cmd2chassis_data_t get_data_from_cmd;
    xQueueReceive(cmd2chassis_queue_handle, &get_data_from_cmd, 0);
    // 从队列获取sensor消息
    sensor2chassis_data_t get_data_from_sensor;
    xQueueReceive(sensor2chassis_queue_handle, &get_data_from_sensor, 0);
    // 控制电机
    if (get_data_from_cmd.mode == normal)
    {
        MotorEnable(&motor_lf1.base);
        MotorEnable(&motor_lb2.base);
        MotorEnable(&motor_rb3.base);
        MotorEnable(&motor_rf4.base);
        ChassisCmd_t cmd = {
            .vx = get_data_from_cmd.vx,
            .vy = get_data_from_cmd.vy,
            .w = get_data_from_cmd.w,
        };
        ChassisPositionLite_Inverse(&chassis, cmd);
    }
    else if (get_data_from_cmd.mode == gyro)
    {
        MotorEnable(&motor_lf1.base);
        MotorEnable(&motor_lb2.base);
        MotorEnable(&motor_rb3.base);
        MotorEnable(&motor_rf4.base);
        vector2_t set_v = {
            .x = get_data_from_cmd.vx,
            .y = get_data_from_cmd.vy};
        // set_v（世界坐标系）顺时针旋转 yaw_angle 到机体坐标系
        float cos_yaw = BSP_Math_Cos(get_data_from_sensor.yaw_angle);
        float sin_yaw = BSP_Math_Sin(get_data_from_sensor.yaw_angle);
        float vx = set_v.x * cos_yaw + set_v.y * sin_yaw;
        float vy = -set_v.x * sin_yaw + set_v.y * cos_yaw;
        ChassisCmd_t cmd = {
            .vx = vx,
            .vy = vy,
            .w = get_data_from_cmd.w,
        };
        ChassisPositionLite_Inverse(&chassis, cmd);
    }
    else
    {
        MotorDisable(&motor_lf1.base);
        MotorDisable(&motor_lb2.base);
        MotorDisable(&motor_rb3.base);
        MotorDisable(&motor_rf4.base);
    }
    // 只要调用一个发送
    MotorSend(&motor_rf4.base);
    // 调试
    MotorData_s md_rb3 = MotorGetData(&motor_rb3.base);
    VofaSetChannel(2, md_rb3.torque_current);
    VofaSetChannel(3, md_rb3.speed);
    VofaSetChannel(4, motor_rb3.base.controller.pid_speed.p_out);
    VofaSetChannel(5, motor_rb3.base.controller.pid_speed.i_out);
    VofaSetChannel(6, motor_rb3.base.controller.pid_speed.d_out);
    VofaSetChannel(7, motor_rb3.base.controller.pid_speed.output);
    VofaSend();
}
