/**
 * @file app_cfg.h
 * @brief APP层配置文件：开发板选择、功能开关、应用参数（target_car）
 * @note 切换开发板：只需修改 DEVELOPMENT_BOARD 宏
 */

#ifndef __APP_CFG_H
#define __APP_CFG_H

#include "bsp_map.h"
#include "robot_def.h"

#define BSP_DWT_USED                  // DWT 高精度定时器（系统时钟基础）
#define BSP_GPIO_USED                 // GPIO 管理 + EXTI 分发
#define BSP_MATH_USED                 // 数学库（向量/矩阵/四元数/三角函数）
#define BSP_FREERTOS_USED             // FreeRTOS 静态创建封装
#define BSP_SPI_USED                  // SPI (BMI088 / LCD)
#define BSP_TIM_USED                  // 定时器 (PWM / 编码器)
#define BSP_USART_USED                // 串口 (SBUS / VOFA / RS485)
#define BSP_CAN_USED                  // CAN (FDCAN / BXCAN, DJI 电机)
#define DRV_PID_USED                  // PID 控制器
#define DRV_MIT_USED                  // MIT PD 控制器
#define DRV_BMI088_USED               // BMI088 IMU 驱动
#define DRV_SBUS_USED                 // SBUS 遥控器驱动
#define DRV_MAHONY_USED               // Mahony 姿态解算
#define DRV_DJIMOTOR_USED             // DJI 电机驱动
#define DRV_DMMOTOR_USED              // DM 电机驱动
#define BMI088_HEAT_USED              // BMI088 加热（TIM8 OPM+RCR 24V 硬件安全关断）
#define DRV_AXIS_MIT_LITE_USED        // 单轴 MIT 关节控制
#define DAEMON_USED                   // Daemon 看门狗
#define VOFA_LITE_USED                // VOFA+ JustFloat 遥测
#define BSP_CRC_USED                  // 软件 CRC 计算
#define LOG_GLOBAL_LIMIT 100          // 日志每秒限量
#define AxisMitVofaLiteSetChannelUsed // 启用轴控制 VOFA 调试通道
#define GENERATE_DISASSEMBLY          // 生成反汇编文件 .lst
#define GENERATE_READELF              // 生成 readelf 输出文件
#define DEVELOPMENT_BOARD DJI_C
#define HAL_CONFIG_NAME DJI_C

// #define UART_LOG_USED     // UART 日志输出

#endif // __APP_CFG_H
