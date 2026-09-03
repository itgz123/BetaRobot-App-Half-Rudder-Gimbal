#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
#include "app_proto_visual.h"
#include "robot_def.h" // gimbal限位/速度/加速度宏
//
#include "drv_dbus.h"
#include "drv_sbus.h"
#include "drv_planner.h"
#include "drv_comm.h"
#include "comm_media_usb_simple.h"
//
#include "bsp_freertos.h"
#include "bsp_assert.h"
#include "lib_math.h"
#include "bsp_dwt.h"
//
#include <string.h>

/*============================================
 *              宏
 *============================================*/
// 摇杆死区：通道值小于该值视为 0，避免中心抖动引起缓慢漂移
#define DEADZONE (0.01f)
#define sbus_half 0.5 // 判断开关通道float等于1/-1/0

/*============================================
 *              枚举
 *============================================*/
typedef enum
{
    no_control_e = 0,
    sbus_e = 1,
    photo_story_e = 2,
    keyboard_mouse_e = 3,
    visual_control_e = 4,
} cmd_control_type; // 控制类型

/*============================================
 *              变量
 *============================================*/
static float pitch_ch, yaw_ch; // sbus，图传，键鼠都是给2个通道的

static cmd2gimbal_data_t cmd_cmd2gimbal_data; // cmd-gimbal
static gimbal2cmd_data_t cmd_gimbal2cmd_data; // gimbal-cmd
static PlannerInstance pitch_planner;         // pitch规划器
static PlannerInstance yaw_planner;           // yaw规划器

static cmd_control_type used_remote_control; // 控制类型

SBUS_INSTANCE_DEF(sbus_inst); // sbus实例

// 视觉的usb虚拟串口
static vision_recv_t vision_recv_data = {0};
static vision_send_t vision_send_data = {0};
/* 视觉通信对话：MEDIA_USB_SIMPLE 短帧免序号（50B/57B ≤ 64B 单包透传），
 * 收发协议 VISUAL（接收 payload 48B / 发送 payload 55B；media 缓冲自动 = 50/57 对齐原帧长）。
 * 48/55 为与视觉电脑约定的线长（协议文档值）：结构与约定的一致性由 COMM_DEF
 * 内部 _Static_assert 编译期校验，不再在此单独断言。
 * 发送由业务层填充 vision_send_t 后 CommSend(&vis_comm, (uint8_t *)&send)。 */
COMM_DEF(vis_comm, MEDIA_USB_SIMPLE, VISUAL, VISUAL, vision_recv_t, 48, vision_send_t, 55, UNPACK_IN_ISR);

/*============================================
 *              私有函数
 *============================================*/
/* 视觉接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，
 * 必须同步拷贝解析）。payload = 48B 帧体（含 cmd_ID），memcpy 到 packed 结构体即得业务字段 */
static void VisionRecvOnFrame(const uint8_t *payload)
{
    memcpy(&vision_recv_data, payload, sizeof(vision_recv_data));
}
/* 板→视觉 状态回传：填充 vision_send_t（55B）后 CommSend。
 * 单位约定：角度字段 deg；角速度 rad/s；速度 m/s。
 * 可得数据取真实值（云台反馈来自 gimbal2cmd 队列）；暂无数据源的字段置 0。 */
static void VisionSend(void)
{
    vision_send_data.cmd_ID = VISUAL_CMD_TX;
    vision_send_data.time_stamp = (uint32_t)(DWT_GetTimeUs() / 1000); /* 板卡时间戳 ms */
    if (visual_control_e == used_remote_control)
    {
        vision_send_data.mode = vision_mode_auto_aim_e;
    }
    else
    {
        vision_send_data.mode = vision_mode_idle_e;
    }
    vision_send_data.yaw = RAD_TO_DEG(cmd_gimbal2cmd_data.yaw_position);
    vision_send_data.pitch_base_relative = RAD_TO_DEG(cmd_gimbal2cmd_data.pitch_position);
    vision_send_data.pitch_down = RAD_TO_DEG(cmd_gimbal2cmd_data.pitch_down_position); // 下pitch位姿角 (rad→deg)
    vision_send_data.yaw_vel = cmd_gimbal2cmd_data.yaw_vel;
    vision_send_data.pitch_base_relative_vel = cmd_gimbal2cmd_data.pitch_vel;
    // 暂无数据：
    vision_send_data.roll = 0;     // IMU roll（暂未通过队列回传）
    vision_send_data.roll_vel = 0; // IMU roll 角速度（暂未回传）
    vision_send_data.v_x = 0;      // 车体 x 速度（暂无底盘反馈）
    vision_send_data.v_y = 0;
    vision_send_data.v_z = 0;
    vision_send_data.bullet_speed = 0; // 弹速（暂无发射反馈）
    vision_send_data.bullet_count = 0; // 弹量（暂无发射反馈）
    vision_send_data.aim_color = 0;    // 瞄准敌方颜色（来自裁判系统）
    CommSend(&vis_comm, (uint8_t *)&vision_send_data);
}
static void sbus_control(void)
{
    cmd_cmd2gimbal_data.state = enable;
    pitch_ch = sbus_inst.sbus_data.ch[2];
    yaw_ch = sbus_inst.sbus_data.ch[3];
}
static void photo_story_control(void)
{
}
static void keyboard_mouse_control(void)
{
}
static void visual_control(void)
{
    cmd_cmd2gimbal_data.state = enable;
    // 单位约定：视觉 pitch/yaw 为 deg（转 rad）；v/a 为 rad/s、rad/s²（不转）
    // 坐标系：pitch 为 base-relative（与 app_gimbal 一致）；yaw 为世界系（需标定对齐电机系）
    cmd_cmd2gimbal_data.pitch_x = DEG_TO_RAD(vision_recv_data.pitch_base_relative);
    cmd_cmd2gimbal_data.pitch_v = vision_recv_data.v_pitch_base_relative;
    cmd_cmd2gimbal_data.pitch_a = vision_recv_data.a_pitch_base_relative;
    cmd_cmd2gimbal_data.yaw_x = DEG_TO_RAD(vision_recv_data.yaw);
    cmd_cmd2gimbal_data.yaw_v = vision_recv_data.v_yaw;
    cmd_cmd2gimbal_data.yaw_a = vision_recv_data.a_yaw;
}
static void planer_control(void)
{
    PlannerInput_s in;
    PlannerOutput_s out;

    pitch_ch = (Lib_Math_Fabs(pitch_ch) < DEADZONE) ? 0.0f : pitch_ch;
    in.current_position = cmd_gimbal2cmd_data.pitch_position;
    in.current_speed = cmd_gimbal2cmd_data.pitch_vel;
    in.current_acceleration = 0.0f; // 电机无加速度反馈
    in.target_cmd = pitch_ch;
    PlannerCalculate(&pitch_planner, &in, &out);
    cmd_cmd2gimbal_data.pitch_x = out.position;
    cmd_cmd2gimbal_data.pitch_v = out.speed;
    cmd_cmd2gimbal_data.pitch_a = out.acceleration;

    yaw_ch = (Lib_Math_Fabs(yaw_ch) < DEADZONE) ? 0.0f : yaw_ch;
    yaw_ch = -yaw_ch; // yaw 已约定逆时针为正（gimbal 端电机方向镜像），此处取反补偿，保持摇杆物理转向不变
    in.current_position = cmd_gimbal2cmd_data.yaw_position;
    in.current_speed = cmd_gimbal2cmd_data.yaw_vel;
    in.current_acceleration = 0.0f;
    in.target_cmd = yaw_ch;
    PlannerCalculate(&yaw_planner, &in, &out);
    cmd_cmd2gimbal_data.yaw_x = out.position;
    cmd_cmd2gimbal_data.yaw_v = out.speed;
    cmd_cmd2gimbal_data.yaw_a = out.acceleration;
}

/*============================================
 *              函数
 *============================================*/
void AppCmdInit(void)
{
    // 注册 SBUS（仅硬件绑定）
    BSP_ASSERT_APP_CALL(SBUSRegister(&sbus_inst));

    // 配置 SBUS（硬件映射 + 运行参数）
    SBUS_Config_s sbus_cfg = {
        .uart_e = UART_SBUS,
        .daemon_reload = 100,
        .daemon_fault = DAEMON_FAULT_NONE,
        .lost_timeout_ms = 1000,
    };
    BSP_ASSERT_APP_CALL(SBUSConfig(&sbus_inst, &sbus_cfg));

    // 初始化规划器（位置限幅/位置模式/最大速度/最大加速度）
    Planner_Init_Config_s pitch_cfg = {
        .position_mode = PLANNER_POS_LIMITED, // pitch 有机械限位 → 限幅
        .pos_limit_min = pitchup_position_1 - pitchup_position_2,
        .pos_limit_max = pitchup_position_3 - pitchup_position_2,
        .max_speed = pitch_speed,
        .max_acc = pitch_acceleration,
    };
    BSP_ASSERT_APP_CALL(PlannerInit(&pitch_planner, &pitch_cfg));

    Planner_Init_Config_s yaw_cfg = {
        .position_mode = PLANNER_POS_WRAP, // yaw 无限旋转 → 归一化
        .pos_limit_min = -M_PI,
        .pos_limit_max = M_PI,
        .max_speed = yaw_speed,
        .max_acc = yaw_acceleration,
    };
    BSP_ASSERT_APP_CALL(PlannerInit(&yaw_planner, &yaw_cfg));

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

ITCM_RAM void AppCmdRun(void)
{
    // 0. 读取云台反馈（规划器需要当前位置/速度）
    xQueueReceive(gimbal2cmd_queue_handle, &cmd_gimbal2cmd_data, 0);

    // 1. 控制源选择：
    used_remote_control = no_control_e;
    // 选择建立在sbus遥控在线的基础上
    if (sbus_inst.daemon->is_online == 1)
    {
        // 这里的选择路径比较复杂
        if (sbus_inst.sbus_data.ch[4] > sbus_half)
        {
            if ((sbus_inst.sbus_data.ch[5] > sbus_half) && (vision_appear_e == vision_recv_data.appear))
            {
                used_remote_control = visual_control_e;
            }
            else
            {
                used_remote_control = sbus_e;
            }
        }
    }

    // 2. 设置要发送的数据（默认失能 + 设定值清零）
    cmd_cmd2gimbal_data.state = disable;
    cmd_cmd2gimbal_data.pitch_x = 0.0f;
    cmd_cmd2gimbal_data.pitch_v = 0.0f;
    cmd_cmd2gimbal_data.pitch_a = 0.0f;
    cmd_cmd2gimbal_data.yaw_x = 0.0f;
    cmd_cmd2gimbal_data.yaw_v = 0.0f;
    cmd_cmd2gimbal_data.yaw_a = 0.0f;
    if (visual_control_e == used_remote_control) // 使用视觉：直接给位置/速度/加速度
    {
        visual_control();
    }
    else
    {
        if (sbus_e == used_remote_control) // 使用sbus遥控：摇杆 -1~1 → 目标速度 → 规划器
        {
            sbus_control();
        }
        else if (photo_story_e == used_remote_control) // 使用图传遥控
        {
            photo_story_control();
        }
        else if (keyboard_mouse_e == used_remote_control) // 使用图传键鼠
        {
            keyboard_mouse_control();
        }
        planer_control(); // 规划期控制：sbus，图传，键鼠都是给2个通道的float
    }

    // 3. 发送出去
    xQueueOverwrite(cmd2gimbal_queue_handle, &cmd_cmd2gimbal_data); // 通过队列
    VisionSend();                                                   // 板→视觉 状态回传（USB 虚拟串口）
}
