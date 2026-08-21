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
#include "bsp_assert.h"
//
#include "drv_comm.h"
#include "comm_media_usb_simple.h"
#include "app_proto_visual.h"
#include <string.h>

static BMI088_Data_t imu = {0};
static euler_t euler = {0};
// static sensor2gimbal_data_t sensor2gimbal_data;
static uint64_t last_imu_ts = 0; /* 上帧 IMU 时间戳 (us)，用于计算 dt */
static float dt;
static vector3_t gyro;
static vector3_t acc;
BMI088_INSTANCE_DEF(bmi088);
MAHONY_INSTANCE_DEF(mahony);

/* 最新视觉接收帧：USB 接收回调同步 memcpy 更新，业务层（云台等）只读引用 */
vision_recv_t vision_recv_data = {0};

/* 视觉通信对话：MEDIA_USB_SIMPLE 短帧免序号（50B/57B ≤ 64B 单包透传），
 * 收发协议 VISUAL（接收 payload 48B / 发送 payload 55B；media 缓冲自动 = 50/57 对齐原帧长）。
 * 发送由业务层填充 vision_send_t 后 CommSend(&vis_comm, (uint8_t *)&send)。 */
COMM_DEF(vis_comm, MEDIA_USB_SIMPLE, VISUAL, VISUAL, 48, 55, UNPACK_IN_ISR);

/* 视觉接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，
 * 必须同步拷贝解析）。payload = 48B 帧体（含 cmd_ID），memcpy 到 packed 结构体即得业务字段 */
static void VisionRecvOnFrame(const uint8_t *payload)
{
    memcpy(&vision_recv_data, payload, sizeof(vision_recv_data));
}

void AppSensorInit(void)
{
    // 注册 BMI088（只注册子模块，Config 时配置硬件）
    BSP_ASSERT_APP_CALL(BMI088Register(&bmi088));

    // 配置 BMI088（硬件枚举 + 传感器参数 + daemon）
    BMI088_Config_s bmi088_cfg = {
        .spi_e = SPI_BMI088,
        .cs_acc_e = GPIO_BMI088_CS_ACCEL,
        .cs_gyro_e = GPIO_BMI088_CS_GYRO,
        .int_acc_e = GPIO_BMI088_INT_ACCEL,
        .int_gyro_e = GPIO_BMI088_INT_GYRO,
        .heater_e = TIM_HEATER,
        .daemon_reload = 20,
        .daemon_fault = DAEMON_FAULT_NONE,
        .acc_range = BMI088_ACC_RANGE_3G,
        .acc_bwp = BMI088_ACC_BWP_NORMAL,
        .acc_odr = BMI088_ACC_ODR_400,
        .gyro_range = BMI088_GYRO_RANGE_2000,
        .gyro_conf = BMI088_GYRO_CONF_2000_230,
        .work_mode = BMI088_MODE_INT,
    };
    BSP_ASSERT_APP_CALL(BMI088Config(&bmi088, &bmi088_cfg));

    // 初始化 Mahony 滤波器
    Mahony_Init_Config_s mahony_cfg = {
        .kp = 0.5f,
        .ki = 0.0f,
    };
    MahonyInit(&mahony, &mahony_cfg);

    // 视觉通信（USB CDC 虚拟串口）：登记协议后端 + 注册/配置 comm。
    // 接收回调同步更新 vision_recv_data；发送由业务层填充 vision_send_t 后 CommSend。
    CommProtoRegisterBackend(&g_visual_backend); /* 登记 VISUAL 后端，须在 CommRegister 之前 */
    CommConfig_s vis_cfg = {
        .media_cfg = &(USB_Config_s){0}, /* USB 无运行期参数（接收钩子由 media 层强制接管） */
        .on_frame = VisionRecvOnFrame,
    };
    CommRegister(&vis_comm);
    CommConfig(&vis_comm, &vis_cfg);
}

ITCM_RAM void AppSensorRun(void)
{
    // 读取 BMI088 原始数据
    imu = BMI088ReadInt(&bmi088);

    // 计算 dt：用 BMI088 插值时间戳之差 (us → s)
    dt = 0.0f;
    if (imu.time_stamp > 0 && last_imu_ts > 0)
    {
        dt = (float)(imu.time_stamp - last_imu_ts) * 1e-6f;
    }
    last_imu_ts = imu.time_stamp;

    // Mahony 姿态解算（dt 由 APP 层根据 BMI088 插值时间戳传入）
    gyro.x = imu.gyro[0];
    gyro.y = imu.gyro[1];
    gyro.z = imu.gyro[2];
    acc.x = imu.acc[0];
    acc.y = imu.acc[1];
    acc.z = imu.acc[2];
    MahonyUpdate(&mahony, gyro, acc, dt);

    // 从 Mahony 四元数解算 yaw 角
    euler = BSP_Math_QuatToEuler(mahony.quat);

    // 通过队列发送出去
    // xQueueOverwrite(sensor2gimbal_queue_handle, &sensor2gimbal_data);
}
