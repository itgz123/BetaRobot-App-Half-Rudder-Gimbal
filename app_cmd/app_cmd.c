#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
#include "app_sensor.h" // vision_recv_data
#include "robot_def.h"  // gimbal限位/速度/加速度宏
//
#include "drv_dbus.h"
#include "drv_planner.h"
//
#include "bsp_freertos.h"
#include "bsp_assert.h"
#include "bsp_math.h"

/*============================================
 *              宏
 *============================================*/
// 摇杆死区：通道值小于该值视为 0，避免中心抖动引起缓慢漂移
#define DEADZONE (0.01f)

/*============================================
 *              枚举
 *============================================*/
typedef enum
{
    no_control_e = 0,
    dbus_e = 1,
    photo_story_e = 2,
    keyboard_mouse_e = 3,
    visual_control_e = 4,
} cmd_control_type; // 控制类型

/*============================================
 *              变量
 *============================================*/
static cmd2gimbal_data_t cmd_cmd2gimbal_data; // cmd-gimbal
static gimbal2cmd_data_t cmd_gimbal2cmd_data; // gimbal-cmd
static PlannerInstance pitch_planner;         // pitch规划器
static PlannerInstance yaw_planner;           // yaw规划器

static cmd_control_type used_remote_control; // 控制类型

DBUS_INSTANCE_DEF(dbus_inst); // dbus实例

/*============================================
 *              私有函数
 *============================================*/
static void dbus_control(void)
{
    if (DBUS_SW_MID == dbus_inst.dbus_data.s1)
    {
        cmd_cmd2gimbal_data.state = enable;
    }
    PlannerInput_s in;
    PlannerOutput_s out;

    float pitch_ch = dbus_inst.dbus_data.ch[3];
    pitch_ch = (BSP_Math_Fabs(pitch_ch) < DEADZONE) ? 0.0f : pitch_ch;
    in.current_position = cmd_gimbal2cmd_data.pitch_position;
    in.current_speed = cmd_gimbal2cmd_data.pitch_vel;
    in.current_acceleration = 0.0f; // 电机无加速度反馈
    in.target_cmd = pitch_ch;
    PlannerCalculate(&pitch_planner, &in, &out);
    cmd_cmd2gimbal_data.pitch_x = out.position;
    cmd_cmd2gimbal_data.pitch_v = out.speed;
    cmd_cmd2gimbal_data.pitch_a = out.acceleration;

    float yaw_ch = dbus_inst.dbus_data.ch[2];
    yaw_ch = (BSP_Math_Fabs(yaw_ch) < DEADZONE) ? 0.0f : yaw_ch;
    in.current_position = cmd_gimbal2cmd_data.yaw_position;
    in.current_speed = cmd_gimbal2cmd_data.yaw_vel;
    in.current_acceleration = 0.0f;
    in.target_cmd = yaw_ch;
    PlannerCalculate(&yaw_planner, &in, &out);
    cmd_cmd2gimbal_data.yaw_x = out.position;
    cmd_cmd2gimbal_data.yaw_v = out.speed;
    cmd_cmd2gimbal_data.yaw_a = out.acceleration;
}
static void photo_story_control(void)
{
}
static void keyboard_mouse_control(void)
{
}
static void visual_control(void)
{
    // cmd_cmd2gimbal_data.state = enable;
    // // 单位约定：视觉 pitch/yaw 为 deg（转 rad）；v/a 为 rad/s、rad/s²（不转）
    // // 坐标系：pitch 为 base-relative（与 app_gimbal 一致）；yaw 为世界系（需标定对齐电机系）
    // cmd_cmd2gimbal_data.pitch_x = DEG_TO_RAD(vision_recv_data.pitch_base_relative);
    // cmd_cmd2gimbal_data.pitch_v = vision_recv_data.v_pitch_base_relative;
    // cmd_cmd2gimbal_data.pitch_a = vision_recv_data.a_pitch_base_relative;
    // cmd_cmd2gimbal_data.yaw_x = DEG_TO_RAD(vision_recv_data.yaw);
    // cmd_cmd2gimbal_data.yaw_v = vision_recv_data.v_yaw;
    // cmd_cmd2gimbal_data.yaw_a = vision_recv_data.a_yaw;
}

/*============================================
 *              函数
 *============================================*/
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
}

ITCM_RAM void AppCmdRun(void)
{
    // 0. 读取云台反馈（规划器需要当前位置/速度）
    xQueueReceive(gimbal2cmd_queue_handle, &cmd_gimbal2cmd_data, 0);

    // 1. 控制源选择：
    //    dbus 在线（未丢帧/失控）时：S1 上=dbus手动，S1 中=视觉，S1 下=失能
    used_remote_control = no_control_e;
    if ((dbus_inst.daemon->is_online == 1) && (dbus_inst.dbus_data.failsafe == 0))
    {
        used_remote_control = dbus_e;
    }

    // 2. 设置要发送的数据（默认失能 + 设定值清零）
    cmd_cmd2gimbal_data.state = disable;
    cmd_cmd2gimbal_data.pitch_x = 0.0f;
    cmd_cmd2gimbal_data.pitch_v = 0.0f;
    cmd_cmd2gimbal_data.pitch_a = 0.0f;
    cmd_cmd2gimbal_data.yaw_x = 0.0f;
    cmd_cmd2gimbal_data.yaw_v = 0.0f;
    cmd_cmd2gimbal_data.yaw_a = 0.0f;

    if (dbus_e == used_remote_control) // 使用dbus遥控：摇杆 -1~1 → 目标速度 → 规划器
    {
        dbus_control();
    }
    else if (photo_story_e == used_remote_control) // 使用图传遥控
    {
        photo_story_control();
    }
    else if (keyboard_mouse_e == used_remote_control) // 使用图传键鼠
    {
        keyboard_mouse_control();
    }
    else if (visual_control_e == used_remote_control) // 使用视觉：直接给位置/速度/加速度
    {
        visual_control();
    }
    else // 全部失效
    {
        cmd_cmd2gimbal_data.state = disable;
    }

    // 3. 通过队列发送出去
    xQueueOverwrite(cmd2gimbal_queue_handle, &cmd_cmd2gimbal_data);
}
