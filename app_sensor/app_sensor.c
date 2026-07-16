#include "app_sensor.h"
#include "app_cfg.h"
#include "app.h"
//
#include "drv_bmi088.h"
#include "drv_mahony.h"
#include "drv_vofa.h"
//
#include "bsp_freertos.h"
#include "bsp_dwt.h"

BMI088_INSTANCE_DEF(bmi088);
MAHONY_INSTANCE_DEF(mahony);
BMI088_Data_t imu = {0};
euler_t euler = {0};
static uint64_t last_imu_ts = 0; /* 上帧 IMU 时间戳 (us)，用于计算 dt */
void AppSensorInit(void)
{
    // 注册并初始化 BMI088
    BMI088_Init_Config_s bmi088_cfg = {
        .spi_e = SPI_BMI088,
        .cs_acc_e = GPIO_BMI088_CS_ACCEL,
        .cs_gyro_e = GPIO_BMI088_CS_GYRO,
        .int_acc_e = GPIO_BMI088_INT_ACCEL,
        .int_gyro_e = GPIO_BMI088_INT_GYRO,
        .heater_e = TIM_HEATER,
        .acc_range = BMI088_ACC_RANGE_3G,
        .acc_bwp = BMI088_ACC_BWP_NORMAL,
        .acc_odr = BMI088_ACC_ODR_400,
        .gyro_range = BMI088_GYRO_RANGE_2000,
        .gyro_conf = BMI088_GYRO_CONF_2000_230,
        .daemon_reload = 20,
        .daemon_fault = DAEMON_FAULT_NONE,
        .work_mode = BMI088_MODE_INT,
    };
    BMI088Register(&bmi088, &bmi088_cfg);

    // 初始化 Mahony 滤波器
    Mahony_Init_Config_s mahony_cfg = {
        .kp = 0.5f,
        .ki = 0.0f,
    };
    MahonyInit(&mahony, &mahony_cfg);
}

ITCM_RAM void AppSensorRun(void)
{
    // 读取 BMI088 原始数据
    imu = BMI088ReadInt(&bmi088);

    // 计算 dt：用 BMI088 插值时间戳之差 (us → s)
    float dt = 0.0f;
    if (imu.time_stamp > 0 && last_imu_ts > 0)
    {
        dt = (float)(imu.time_stamp - last_imu_ts) * 1e-6f;
    }
    last_imu_ts = imu.time_stamp;

    // Mahony 姿态解算（dt 由 APP 层根据 BMI088 插值时间戳传入）
    vector3_t gyro = {imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    vector3_t acc = {imu.acc[0], imu.acc[1], imu.acc[2]};
    MahonyUpdate(&mahony, gyro, acc, dt);

    // 从 Mahony 四元数解算 yaw 角
    euler = BSP_Math_QuatToEuler(mahony.quat);

    // 给底盘发送 yaw 轴数据
    sensor2chassis_data_t send_data = {
        .yaw_rate = imu.gyro[2], // yaw 角速度 (rad/s)
        .yaw_angle = euler.yaw,  // Mahony 解算的 yaw 角 (rad)
        .yaw_acc = imu.acc[0],   // 机体 x 轴加速度 (m/s²)
    };
    xQueueOverwrite(sensor2chassis_queue_handle, &send_data);
    VofaSetChannel(14, bmi088.temperature);
}
