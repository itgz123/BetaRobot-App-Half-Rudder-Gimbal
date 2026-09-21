/**
 * @file app_cfg.h
 * @brief APP层配置文件：开发板选择、功能开关、应用参数（target_car）
 * @note 切换开发板：只需修改 DEVELOPMENT_BOARD 宏
 */

#ifndef __APP_CFG_H
#define __APP_CFG_H

#include "bsp_map.h"
#include "robot_def.h"

/* USED开关 */
#define BSP_SYS_STATUS_USED         // 系统状态计数（app 初始化错误 / FreeRTOS 任务超时）
#define BSP_DWT_USED                // DWT 高精度定时器（系统时钟基础）
#define BSP_GPIO_USED               // GPIO 管理 + EXTI 分发
#define LIB_MATH_USED               // 数学库（向量/矩阵/四元数/三角函数）
#define LIB_MATH_TRIG_LUT_USED      // 自研查表三角函数 Lib_Math_*LUT（不定义则表不编译、接口不可用）
#define BSP_FREERTOS_USED           // FreeRTOS 静态创建封装
#define BSP_FREERTOS_STATUS_USED    // FreeRTOS 运行状态（钩子 + 空闲任务状态快照，调试器读 bsp_freertos_status）
#define BSP_SPI_USED                // SPI (BMI088 / LCD)
#define BSP_TIM_USED                // 定时器 (PWM / 编码器)
#define BSP_USART_USED              // 串口 (SBUS / VOFA / RS485)
#define BSP_CAN_USED                // CAN (FDCAN / BXCAN, DJI 电机)
#define BSP_CAN_LIST_LUT_USED       // CAN 接收中断标准ID LIST模式按ID查表加速（不定义则回退循环判断）
#define LIB_PID_USED                // PID 控制器
#define LIB_MIT_USED                // MIT PD 控制器
#define DRV_BMI088_USED             // BMI088 IMU 驱动
#define DRV_DBUS_USED               // DBUS 遥控器驱动
#define DRV_SBUS_USED               // SBUS 遥控器驱动
#define DRVLIB_BMI088_KALMAN_USED   // BMI088 零偏标定 + 线性卡尔曼姿态（drv_bmi088 + lib_kf 联合）
#define LIB_KF_USED                 // 通用卡尔曼滤波 (lib_kf)
#define DRV_DJIMOTOR_BROADCAST_USED // DJI 电机驱动
#define DRV_DMMOTOR_USED            // DM 电机驱动
#define DRVS_DMMOTOR_USED            // DM 电机驱动（drvs 纯协议版，重构中，暂未被 app 引用）
#define DRVS_RSMOTOR_USED            // RS 电机驱动（drvs 纯协议版，重构中，暂未被 app 引用）
#define DRVS_LKMOTOR_USED            // LK 电机驱动（drvs 纯协议版一对一，重构中，暂未被 app 引用）
#define DRVS_DJIMOTOR_BROADCAST_USED // DJI 电机驱动（drvs 纯协议版一拖四广播，重构中，暂未被 app 引用）
#define DRVS_LKMOTOR_BROADCAST_USED  // LK 电机驱动（drvs 纯协议版一拖四广播，重构中，暂未被 app 引用）
#define DRV_RSMOTOR_USED            // RS05 电机驱动（灵足时代，MIT 协议）
#define DRV_AXIS_MIT_LITE_USED      // 单轴 MIT 关节控制
#define DAEMON_USED                 // Daemon 看门狗
#define VOFA_USED                   // VOFA+ JustFloat 遥测
#define DRV_TERMINAL_LITE_USED      // 串口调参终端（terminal_lite，需 TERMINAL_LITE_UART）
#define LIB_CRC_USED                // 软件 CRC 计算（Direct/GenTable/TableCalc）
#define LIB_CRC_TABLES_USED         // 软件 CRC 常用算法 Flash 表（lib_crc_tables.c）
#define LIB_HAMMING_USED            // 汉明码纠错（标准 / 扩展缩短 SECDED，任意 bit 长度）
#define DRV_COMM_USED               //
#define LIB_FORMAT_USED             // 快速格式化（零除法整数转换，bsp_log 依赖）
#define BSP_LOG_USED                // 日志输出
//
/* TODO 加热器已从 drv_bmi088 移出、不再参与编译（IMU 不加热 → 不存在热坏风险）。
 *      后续独立为 drv_heater：温度取自 BMI088GetTemperature，控温由 app 任务驱动，
 *      TIM8 OPM+RCR 安全链下沉 bsp_tim；届时在此加 DRV_HEATER_USED 开关。
 *      旧实现（含 8 层安全设计）见 git 历史 drv/drv_bmi088/drv_bmi088_heater.c */
//
#define GENERATE_DISASSEMBLY // 生成反汇编文件 .lst
#define GENERATE_READELF     // 生成 readelf 输出文件

/* 开发板 */
#define DEVELOPMENT_BOARD DJI_C
#define HAL_CONFIG_NAME DJI_C
/* UART选择 */
#if DEVELOPMENT_BOARD == DM_MC02
#define VOFA_UART UART_1 // UART_RS485_2
#define LOG_UART UART_7
#define TERMINAL_LITE_UART UART_10 // terminal_lite 调参串口（空闲且 RX/TX DMA 已配）
#elif DEVELOPMENT_BOARD == DJI_C   // UART_1:4pin,UART_6:3pin
#define VOFA_UART UART_6
// #define LOG_UART UART_6
#define TERMINAL_LITE_UART UART_1 // 若需用 terminal 调参，取消注释（需接 USB-TTL）
#elif DEVELOPMENT_BOARD == DJI_A
// #define VOFA_UART
// #define LOG_UART
#else
#error "error"
#endif

/* 其他 */
#define TERMINAL_LITE_TASK_PRIORITY 2
#define LIB_MATH_TRIG_LUT_SPEED 1 // LUT 速度：0=四分之一表(省flash/象限映射) 1=2π全周期表(无象限映射/更快)
#define LIB_MATH_TRIG_LUT_PREC 3  // LUT 精度：0=低 1=中 2=高 3=满精度(误差<ε=2^-23)

/* 关闭 */

#endif // __APP_CFG_H
