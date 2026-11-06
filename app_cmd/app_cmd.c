#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
#include "app_cmd_chassis.h" // 底盘-云台 CAN 线协议数据结构
#include "app_cmd_gimbal.h"  // gimbal2cmd / cmd2gimbal 队列句柄与数据结构
#include "app_cmd_shoot.h"   // cmd2shoot / shoot2cmd 队列句柄与数据结构
#include "app_cmd_visual.h"  // 视觉线协议业务数据结构
#include "app_proto_visual.h"
#include "robot_def.h" // gimbal限位/速度/加速度宏
//
#include "drv_dbus.h"
#include "drv_sbus.h"
#include "drv_planner.h"
#include "drv_comm.h"
#include "drv_vofa.h"
#include "comm_media_usb_simple.h"
#include "comm_media_can_idseq.h"
#include "comm_proto_ext.h"
//
#include "bsp_freertos.h"
#include "bsp_sys_status.h"
#include "lib_math.h"
#include "bsp_dwt.h"
//
#include <string.h>
/*
底盘和云台都需要添加yaw轴前馈：
chassis：ch3通道控制云台旋转，但是底盘需要等和云台出现相对角度误差了才旋转，导致出现延迟。
         （已实现：云台侧宏 chassis_follow_ff 算速度前馈，底盘侧 PID 把角度误差转成速度）
gimbal：底盘旋转导致云台底座旋转，需要前馈补偿，不然等出现误差再纠正就延迟了。（先不加，做todo）
 */

/*============================================
 *              宏
 *============================================*/
// 摇杆死区：通道值小于该值视为 0，避免中心抖动引起缓慢漂移
#define DEADZONE (0.01f)
#define sbus_half 0.5 // 判断开关通道float等于1/-1/0

/* 发射模式在 ch9 上的五段分界：标准五档开关停位约 -1 / -0.5 / 0 / +0.5 / +1，
 * 依次为 关闭摩擦轮 / 单发 / 三连发 / 连发 / 退弹（预留）。分界取相邻停位中点 ±0.75 / ±0.25。
 * 开关不回中（两端都是有效停位），故按等分取，不设回中死区。 */
#define FIRE_MODE_SEG_1 (-0.75f) // 关闭摩擦轮 | 单发
#define FIRE_MODE_SEG_2 (-0.25f) // 单发 | 三连发
#define FIRE_MODE_SEG_3 (0.25f)  // 三连发 | 连发
#define FIRE_MODE_SEG_4 (0.75f)  // 连发 | 退弹

/* 底盘驱动档在 ch8 上的四段分界：标准四档开关停位约 -1 / -1/3 / +1/3 / +1，
 * 依次为 普通+额外功率 / 普通 / 自由 / 底盘失能。分界取相邻停位中点 0 / ±2/3。
 * 若实车开关停位不同（例如 -1/-0.5/+0.5/+1），按实测通道值改这三个阈值即可。 */
#define CHASSIS_DRIVE_SEG_1 (-2.0f / 3.0f) // 普通+额外功率 | 普通
#define CHASSIS_DRIVE_SEG_2 (0.0f)         // 普通 | 自由
#define CHASSIS_DRIVE_SEG_3 (2.0f / 3.0f)  // 自由 | 底盘失能

/* 分级调试开关（位掩码，宏定义见 app.h）：某一位为 1 表示该子系统参与控制。
 * 未置位的子系统在这里下发"零输出"：
 *   gimbal  → mode=stop（三轴零力矩，电机仍保持使能）
 *   shoot   → friction_mode=disable + fire_mode=off（摩擦轮与拨盘都零电流，app_shoot.c 同样看该位）
 *   chassis → 失能 + 三轴速度 0（链路照常发送）
 *   referee → 预留给裁判系统
 * ⚠ CMD_DEBUG_STAGE 本身是无括号的 | 组合，做位与时必须先加括号，
 *   否则 | 与 & 的优先级会让判断结果出错。 */
#define CMD_DEBUG_ENABLED(stage) (((CMD_DEBUG_STAGE) & (stage)) != 0)
/* 云台未使能 → 底盘直通：没有云台指向可追，底盘不再做 θ 旋转/跟随，
 * w 改由空出来的云台 yaw 摇杆直控。 */
#define CMD_DEBUG_STAGE_CHASSIS_DIRECT (!CMD_DEBUG_ENABLED(CMD_DEBUG_STAGE_gimbal))

/* SBUS 通道分配（0 基；摇杆 -1~1，三档开关 -1/0/+1，旋钮 -1~1）
 * 注：README「操作逻辑」一节写的拨杆号与代码不一致，这里以实机在用的映射为准。 */
#define SBUS_CH_VY 0            // 右摇杆左右 → 底盘 vy
#define SBUS_CH_VX 1            // 右摇杆前后 → 底盘 vx
#define SBUS_CH_PITCH 2         // 左摇杆上下 → 云台 pitch
#define SBUS_CH_YAW 3           // 左摇杆左右 → 云台 yaw
#define SBUS_CH_ENABLE 4        // 总开关：上 = 使能
#define SBUS_CH_VISION 5        // 视觉接管允许（两段）：上 = 允许视觉接管云台（仍需视觉报有目标）
#define SBUS_CH_MODE 6          // 档位开关（三档）：hole / normal / gyro
#define SBUS_CH_FIRE 7          // 开火总闸（两段开关）：0 = 关，1 = 由下面这个旋钮选模式
#define SBUS_CH_CHASSIS_DRIVE 8 // 底盘驱动档（四段开关）：普通+额外功率 / 普通 / 自由 / 底盘失能
#define SBUS_CH_FIRE_MODE 9     // 发射模式五档开关：关闭摩擦轮 / 单发 / 三连发 / 连发 / 退弹（非回中，无死区）

/* 视觉直通路的限幅（cmd 不过规划器，直接把视觉给的 位置/速度/加速度 下发给云台）。
 * 数值与摇杆规划器是同一套整定值（robot_def.h 的 pitch_speed 等），但用独立宏名：
 * 视觉是外环直接给设定值，将来若要单独收紧/放宽，不必改动遥控手感。
 * pitch 位置限位与规划器同坐标：0 = 上 pitch 水平（下 pitch 直立）。 */
#define VISION_PITCH_POS_LIMIT_MIN (pitchup_position_1 - pitchup_position_2)
#define VISION_PITCH_POS_LIMIT_MAX (pitchup_position_3 - pitchup_position_2)
/* 内部 pitch 坐标（gimbal 回传的 pitch_position）在“下 pitch 直立、上 pitch 水平”时的
 * 数值不是 0：pitch_position = (u - U0) - (d - Dmin)，代入标定姿态即为
 * (U2 - U0) - (Dmax - Dmin) ≈ -0.0912 rad(-5.23°)。
 * 视觉 pitch_base_relative 以“上 pitch 水平 = 0”为原点、向上为负，
 * 故内部位置设定值 = -视觉角度 + 本偏置；回传时反过来先减掉本偏置再取反。 */
#define VISION_PITCH_ZERO_OFFSET ((pitchup_position_2 - pitchup_position_0) - (pitchdown_position_max - pitchdown_position_min))
#define VISION_PITCH_SPEED_LIMIT (pitch_speed)
#define VISION_PITCH_ACC_LIMIT (pitch_acceleration)
#define VISION_YAW_SPEED_LIMIT (yaw_speed)
#define VISION_YAW_ACC_LIMIT (yaw_acceleration)

/* 原 ch8 速度旋钮已改作底盘驱动档：底盘平动与小陀螺速度固定取旋钮中位，
 * 即归一化通道值 0（knob_scale(0) = 速度区间中点），不再随遥控变化。 */
#define SBUS_SPEED_CHANNEL_FIXED 0.0f

/*============================================
 *              枚举
 *============================================*/
typedef enum : uint8_t
{
    no_control_e = 0,
    sbus_e = 1,
    photo_story_e = 2,
    keyboard_mouse_e = 3,
    // visual_control_e独立于上面模式，也就是说visual_control_e位与上述4种控制方式
    visual_control_e = 4,
} cmd_control_type; // 控制类型

/* 底盘驱动档（ch8 四段开关）：只影响下发给底盘的 enabled/vx/vy/w/yaw。 */
typedef enum : uint8_t
{
    chassis_drive_boost_e = 0,   // 第 1 段：普通行驶 + 额外功率
    chassis_drive_normal_e = 1,  // 第 2 段：普通行驶（跟随云台）
    chassis_drive_free_e = 2,    // 第 3 段：自由模式（不跟随云台，vx/vy 按车身系直通）
    chassis_drive_disable_e = 3, // 第 4 段：底盘失能（云台不受影响）
} chassis_drive_mode_e;

/* AppCmdRun 的帧内上下文：{sbus遥控，图传，键鼠，视觉} 统一接口控制 {云台，底盘，发射，视觉}。
 *
 * 输入源（sbus / 图传 / 键鼠）只负责把自身输入翻译成这里的 归一化通道(-1~1)，
 * 外加 状态机/开火/发射模式 三个开关量；下游（云台规划、底盘解算、开火、状态回传）只认这份
 * 上下文，不再各自去读某个遥控源。视觉源是特例：不写通道，直接给出云台 位置/速度/加速度。 */
typedef struct
{
    cmd_control_type type; // 本帧生效的控制源

    robot_mode mode;                    // 状态机：stop = 失能，normal/gyro/hole 由档位开关解出
    uint8_t fire;                       // 开火总闸（ch7）：0 = 关
    chassis_drive_mode_e chassis_drive; // 底盘驱动档（ch8 四段）
    uint8_t power_boost_enable;         // 额外功率（ch8 第 1 段）：0 = 关；随控制帧转发给底盘
    /* ch9 五档选择器停在"关闭摩擦轮"段（与 ch7 无关，始终生效）。
     * fire_mode 编码不了这一档：它表达的是"关火"，而关闭摩擦轮是另一件事
     * （ch7 断开时轮子本来也允许转，见 send_shoot）。 */
    uint8_t friction_off;
    fire_mode_e fire_mode; // 发射模式：fire=0 → 关；fire=1 → ch9 五段解出 单发/三连发/连发/退弹
    float fire_rate;       // 射频 (发/s)：视觉源由 enable_pitch_diff 换算；遥控源暂无来源

    // 归一化通道 (-1~1)：摇杆/开关中位 0
    float gimbal_pitch_channel;  // 云台 pitch
    float gimbal_yaw_channel;    // 云台 yaw
    float chassis_vx_channel;    // 底盘前后
    float chassis_vy_channel;    // 底盘左右
    float chassis_speed_channel; // 速度档位：现固定为 0（原 ch8 旋钮已改作底盘驱动档），
                                 // 0 = 区间中点，同时缩放平动最大速度与 gyro 档自转角速度
} AppCmdRun_ctx_struct;

/*============================================
 *              变量
 *============================================*/
static cmd2gimbal_data_t cmd_cmd2gimbal_data;           // cmd-gimbal
static gimbal2cmd_data_t cmd_gimbal2cmd_data;           // gimbal-cmd
static cmd2shoot_data_t cmd_cmd2shoot_data;             // cmd-shoot
static shoot2cmd_data_t cmd_shoot2cmd_data;             // shoot-cmd
static PlannerInstance pitch_planner;                   // pitch规划器
static PlannerInstance yaw_planner;                     // yaw规划器
SBUS_INSTANCE_DEF(sbus_inst);                           // sbus实例
static vision_recv_t vision_recv_data = {0};            // 视觉->cmd
static vision_send_t vision_send_data = {0};            // cmd->视觉
static chassis2gimbal_data_t chassis2gimbal_data = {0}; // 底盘→云台（on_frame 同步拷贝）
static gimbal2chassis_data_t gimbal2chassis_data = {0}; // 云台→底盘（业务填写后 CommSend）
/* 视觉通信对话：MEDIA_USB_SIMPLE 短帧免序号（50B/57B ≤ 64B 单包透传），
 * 收发协议 VISUAL（接收 payload 48B / 发送 payload 55B；media 缓冲自动 = 50/57 对齐原帧长）。
 * 48/55 为与视觉电脑约定的线长（协议文档值）：结构与约定的一致性由 COMM_DEF
 * 内部 _Static_assert 编译期校验，不再在此单独断言。
 * 发送由业务层填充 vision_send_t 后 CommSend(&vis_comm, (uint8_t *)&send)。 */
COMM_DEF(vis_comm, MEDIA_USB_SIMPLE, VISUAL, VISUAL, vision_recv_t, 48, vision_send_t, 55, UNPACK_IN_ISR);
/* 底盘通信对话：CAN1 / IDSEQ / PROTO_EXT（帧 = [0xA5][seq][汉明编码区 E][CRC8][0x5A]）。
 * 在 CUSTOM 的定界 + CRC 之上先过一级扩展缩短汉明码（SECDED，m=7）：每块纠 1 位、检 2 位，
 * CAN 差分对上的偶发位翻转不再直接丢掉这一拍控制帧；CRC 仍保留，用来复核汉明的纠错结果
 * （见 comm_proto_ext.h）。分块与帧长全在编译期定死，运行期只按已算好的 cfg 逐块编解码。
 * 云台侧 tx_id=0x100 段、rx_id=0x110 段（底盘侧对调）；ID 段与 CAN1 现有占用
 * （yaw RS05 0x001/0x0FD）及后续 3508 波盘（0x1FF/0x200/0x201~0x208）均不重叠。
 * 收发 payload 分别为 gimbal2chassis_data_t(18B) / chassis2gimbal_data_t(12B)：
 *   18B → 2 块 × 10B 编码区，整帧 24B（3 个 CAN 包）
 *   12B → 1 块 × 13B 编码区，整帧 17B（3 个 CAN 包）
 * CAN1 上挂有经典 CAN 的 RS05，故 mode 必须 CLASSIC（不可用 FD）。 */
COMM_DEF(chassis_comm, MEDIA_CAN_IDSEQ, EXT, EXT, chassis2gimbal_data_t, 12, gimbal2chassis_data_t, 18, UNPACK_IN_ISR);

// AppCmdRun内部上下文
static AppCmdRun_ctx_struct cmd_ctx;
/* 上一拍生效的控制源：用于识别「视觉 → 通道源」的交还边沿，在其首拍给规划器累加器播种。
 * 视觉期间 send_gimbal 走的是直通分支、planner 不被调用，累加器停在旧值上，
 * 交还时不重新锚定会让云台目标瞬间跳到旧值。 */
static cmd_control_type cmd_last_control_type = no_control_e;
/* 本拍云台 yaw 轴的指令角速度 (rad/s，世界系、逆时针为正)：摇杆源取规划器输出的轨迹速度
 * （含加速度限幅，就是云台真正要执行的角速度），视觉源取视觉下发的 v_yaw。
 * 供底盘跟随前馈使用（见 chassis_w_from_mode）。
 * 依赖 AppCmdRun 里 send_gimbal 先于 send_chassis 调用：两者同拍，读到的就是本拍的指令。 */
static float cmd_gimbal_yaw_rate_cmd = 0.0f;

/*============================================
 *              私有函数
 *============================================*/
/*======== 回调函数 =========*/
/* 视觉接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，
 * 必须同步拷贝解析）。payload = 48B 帧体（含 cmd_ID），memcpy 到 packed 结构体即得业务字段 */
static void VisionRecvOnFrame(const uint8_t *payload)
{
    memcpy(&vision_recv_data, payload, sizeof(vision_recv_data));
}

/* 底盘接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，须同步拷贝） */
static void ChassisRecvOnFrame(const uint8_t *payload)
{
    memcpy(&chassis2gimbal_data, payload, sizeof(chassis2gimbal_data));
}

/*======== 输入层：各控制源 → 统一上下文 =========*/
// 通道死区：中位附近的抖动直接归零，避免云台/底盘缓慢漂移
static float channel_deadzone(float ch)
{
    return (Lib_Math_Fabs(ch) < DEADZONE) ? 0.0f : ch;
}

/* 旋钮调速：归一化通道 (-1~1) 线性映射到 [v_min, v_max]
 *   ch = -1 → v_min（旋钮最小档）
 *   ch =  0 → 区间中点
 *   ch = +1 → v_max（旋钮最大档）
 * 旋钮不回中，故调用方直接用通道原值，不套 channel_deadzone。 */
static inline float knob_scale(float ch, float v_min, float v_max)
{
    return (v_max - v_min) * (ch + 1.0f) * 0.5f + v_min;
}

// SBUS 摇杆 → 4 通道
static void input_sbus(void)
{
    cmd_ctx.gimbal_pitch_channel = sbus_inst.sbus_data.ch[SBUS_CH_PITCH];
    cmd_ctx.gimbal_yaw_channel = sbus_inst.sbus_data.ch[SBUS_CH_YAW];
    cmd_ctx.chassis_vx_channel = sbus_inst.sbus_data.ch[SBUS_CH_VX];
    cmd_ctx.chassis_vy_channel = sbus_inst.sbus_data.ch[SBUS_CH_VY];
    // ch8 已改作视觉开关：速度固定取旋钮中位（通道值 0），不再随遥控变化
    cmd_ctx.chassis_speed_channel = SBUS_SPEED_CHANNEL_FIXED;
}

/* 图传遥控 → 4 通道
 * TODO 图传链路（UART）尚无驱动与协议，接入后在 body 里按 input_sbus 的约定填 4 通道，
 *      并补上 状态机/开火 两个开关量；其余流程无需改动。 */
static void input_photo_story(void)
{
}

/* 图传键鼠 → 4 通道
 * TODO 键鼠走图传链路，尚无线协议：鼠标 x/y 增量 → 云台 pitch/yaw，
 *      W/S/A/D → 底盘 vx/vy，左键 → 开火。 */
static void input_keyboard_mouse(void)
{
}

/* 选源 + 取通道：本帧谁在控，以及它的输入怎么变成 4 通道 */
static void input_update(void)
{
    // 默认：无控制源 → 失能停车，4 通道回中位
    cmd_ctx.type = no_control_e;
    cmd_ctx.mode = robot_mode_stop;
    cmd_ctx.fire = 0;
    cmd_ctx.chassis_drive = chassis_drive_normal_e;
    cmd_ctx.power_boost_enable = 0;
    cmd_ctx.friction_off = 0;
    cmd_ctx.fire_mode = fire_mode_off_e;
    cmd_ctx.fire_rate = 0.0f;
    cmd_ctx.gimbal_pitch_channel = 0.0f;
    cmd_ctx.gimbal_yaw_channel = 0.0f;
    cmd_ctx.chassis_vx_channel = 0.0f;
    cmd_ctx.chassis_vy_channel = 0.0f;
    cmd_ctx.chassis_speed_channel = 0.0f; // 中位 = 速度区间中点（失能时无意义）

    /* 源选择暂时只建立在 SBUS 遥控在线的基础上：遥控掉线 → 全部失能。
     * 图传/键鼠接入后，在这里追加各自的在线判断与优先级。 */
    if (sbus_inst.daemon->is_online != 1)
        return;
    if (sbus_inst.sbus_data.ch[SBUS_CH_ENABLE] <= sbus_half)
        return; // 总开关断开 → 失能停车

    // 总开关已打开：解出状态机与开火
    // 档位开关（三档）→ 状态机。从上到下：-1 = hole（上档）、
    // 0 = normal（中档）、+1 = gyro（下档）。
    // 若实车拨杆方向相反，交换首尾两个判断即可。
    float sw = sbus_inst.sbus_data.ch[SBUS_CH_MODE];
    if (sw < -sbus_half)
        cmd_ctx.mode = robot_mode_hole; // 上档
    else if (sw > sbus_half)
        cmd_ctx.mode = robot_mode_gyro; // 下档
    else
        cmd_ctx.mode = robot_mode_normal; // 中档

    /* 底盘驱动档（ch8，四段）：-1 端 → +1 端依次为
     *   普通+额外功率 / 普通 / 自由模式 / 底盘失能。
     * 额外功率只是第 1 段的附加属性，转发给底盘；总开关关闭/遥控掉线时默认普通档。 */
    float cd = sbus_inst.sbus_data.ch[SBUS_CH_CHASSIS_DRIVE];
    if (cd < CHASSIS_DRIVE_SEG_1)
        cmd_ctx.chassis_drive = chassis_drive_boost_e;
    else if (cd < CHASSIS_DRIVE_SEG_2)
        cmd_ctx.chassis_drive = chassis_drive_normal_e;
    else if (cd < CHASSIS_DRIVE_SEG_3)
        cmd_ctx.chassis_drive = chassis_drive_free_e;
    else
        cmd_ctx.chassis_drive = chassis_drive_disable_e;
    cmd_ctx.power_boost_enable = (chassis_drive_boost_e == cmd_ctx.chassis_drive) ? 1u : 0u;

    /* 发射：ch9 是五档选择器（-1 端 → +1 端）：关闭摩擦轮 / 单发 / 三连发 / 连发 / 退弹；
     * ch7 是开火总闸（两段开关），只有 ch7 = 1 时选择的 单发/三连发/连发/退弹 才真正下发。
     * "关闭摩擦轮"段不受 ch7 影响、始终生效 —— 它是摩擦轮的总关，与是否发弹无关，
     * 具体在 send_shoot 里优先于直立/过洞判断。 */
    float fm = sbus_inst.sbus_data.ch[SBUS_CH_FIRE_MODE];
    fire_mode_e fire_sel;
    cmd_ctx.friction_off = 0;
    if (fm < FIRE_MODE_SEG_1)
    {
        fire_sel = fire_mode_off_e;
        cmd_ctx.friction_off = 1;
    }
    else if (fm < FIRE_MODE_SEG_2)
        fire_sel = fire_mode_single_e;
    else if (fm < FIRE_MODE_SEG_3)
        fire_sel = fire_mode_triple_e;
    else if (fm < FIRE_MODE_SEG_4)
        fire_sel = fire_mode_auto_e;
    else
        fire_sel = fire_mode_unjam_e; // 退弹：shoot 侧复用规划器反向走一整格

    cmd_ctx.fire = (sbus_inst.sbus_data.ch[SBUS_CH_FIRE] > sbus_half) ? 1 : 0;
    cmd_ctx.fire_mode = (cmd_ctx.fire != 0) ? fire_sel : fire_mode_off_e;

    // 云台控制权：视觉允许开关（ch5）打开且视觉有目标 → 交给视觉，否则由摇杆控制
    if ((sbus_inst.sbus_data.ch[SBUS_CH_VISION] > sbus_half) && (vision_appear_e == vision_recv_data.appear))
        cmd_ctx.type = visual_control_e;
    else
        cmd_ctx.type = sbus_e;

    // 取 4 通道：目前实机只有 SBUS 一路，图传/键鼠接入后由这里分流
    switch (cmd_ctx.type)
    {
    case sbus_e:
    case visual_control_e: // 视觉只接管云台，底盘平移仍由摇杆给
        input_sbus();
        break;
    case photo_story_e:
        input_photo_story();
        break;
    case keyboard_mouse_e:
        input_keyboard_mouse();
        break;
    default:
        break;
    }

    /* 视觉接管云台时，发弹也交给视觉（覆盖上面由 ch7/ch9 解出的结果）：
     *   enable_yaw_diff   = 0 不发弹 / 1 发弹（连发）→ 视觉没有"单发/三发"概念，按连发处理；
     *   enable_pitch_diff = 弹频档 1/2/3 → 2/4/6 发/s，换算进 fire_rate 供 shoot 侧使用。
     * 视觉无目标/开关关掉时 type 退回 sbus，ch7 的发射控制自动恢复。 */
    if (visual_control_e == cmd_ctx.type)
    {
        if (vision_recv_data.enable_yaw_diff > 0.5f)
        {
            float rate = vision_recv_data.enable_pitch_diff * 2.0f; // 档位 → 发/s
            cmd_ctx.fire = 1;
            cmd_ctx.fire_mode = fire_mode_auto_e;
            cmd_ctx.fire_rate = Lib_Math_Clamp(rate, 0.0f, 6.0f);
        }
        else
        {
            cmd_ctx.fire = 0;
            cmd_ctx.fire_mode = fire_mode_off_e;
            cmd_ctx.fire_rate = 0.0f;
        }
    }

    /* "关闭摩擦轮"段是机械安全选择：无论输入源（ch7 / 视觉）怎么要求，都强制停火，
     * 避免摩擦轮不转时拨盘硬推弹丸。摩擦轮模式在 send_shoot 里同样被该位压成 disable。 */
    if (cmd_ctx.friction_off)
    {
        cmd_ctx.fire = 0;
        cmd_ctx.fire_mode = fire_mode_off_e;
        cmd_ctx.fire_rate = 0.0f;
    }
}

/*======== 处理层：统一上下文 → 各执行器设定值 =========*/
/* 底盘自转的**云台侧前馈** (rad/s)。完整跟随律拆成两半：
 *   云台侧（本文件）：随帧发 w_ff 与目标偏角 θ_ref
 *   底盘侧（app_chassis_force.c 的 ChassisWCommand）：
 *       实测 θ 由底盘直接读共享 CAN1 上的 RS05 得到，
 *       w_out = w_ff + kp × wrap(θ_实测 − θ_ref)
 *
 * 为什么前馈和目标角留在云台：ω_yaw 是云台 yaw 轴的指令角速度，只有本板知道
 * （规划器输出或视觉下发）。反馈 PID 只依赖底盘自己读到的最新 θ，下放到底盘
 * 可以减少一次"编码器 → cmd → CAN → 底盘"的往返。
 *
 *   normal / hole：w_ff = ff × ω_yaw，θ_ref = 0（把相对角拉回 0）
 *   gyro         ：w_ff = 定值自转角速度 spin，θ_ref 以 -spin 斜坡变化
 *                  （见 chassis_yaw_target_from_mode）
 *   stop         ：0
 *
 * 符号：ω_yaw 与底盘 w 都是"逆时针为正"，ff 必须为正。
 * 标定时和底盘侧 CHASSIS_FOLLOW_KP/KI 一起看：前馈与 PID 必须同号，否则是在帮倒忙。 */
static inline float chassis_w_from_mode(float yaw_rate_cmd)
{
    if (robot_mode_gyro == cmd_ctx.mode)
    {
        // 定值按固定的中位缩放值恒等输出（原速度旋钮已改作视觉开关）。
        // 底盘侧 gyro 档只限幅 PID 输出，不压这个自转前馈
        return knob_scale(cmd_ctx.chassis_speed_channel, chassis_gyro_rotate_speed_min, chassis_gyro_rotate_speed);
    }

    if ((robot_mode_normal != cmd_ctx.mode) && (robot_mode_hole != cmd_ctx.mode))
        return 0.0f; // stop：失能，w 给 0

    // 前馈：跟着云台的指令角速度一起转，底盘不等误差、起步即同步。
    // 不受死区影响——云台静止时 ω_yaw 自然是 0，死区由底盘侧 PID 配置管
    return chassis_follow_ff * yaw_rate_cmd;
}

/* gyro 档目标偏角的积分状态。进入 gyro 的首拍用当前实测 θ 播种：目标若从 0
 * 直接起步，会给底盘一个"把相对角拉回 0"的额外冲击。 */
static float chassis_yaw_target = 0.0f;
static robot_mode chassis_yaw_target_mode = robot_mode_stop;

/* 云台→底盘的**目标相对角** θ_ref (rad)：θ = ψ_云台 − ψ_底盘，逆时针为正。
 *   normal/hole：0
 *   gyro       ：以实测 θ 为初值，按 -spin 斜坡（底盘逆时针自转时 θ 以 -spin 减小）
 *   stop/其他  ：0
 * dt 只用于 gyro 积分；模式取 cmd_ctx.mode，与帧里的 enabled 同源。 */
static inline float chassis_yaw_target_from_mode(float dt_s)
{
    const robot_mode mode = cmd_ctx.mode;

    if ((robot_mode_normal == mode) || (robot_mode_hole == mode))
    {
        chassis_yaw_target_mode = mode;
        chassis_yaw_target = 0.0f;
        return 0.0f;
    }

    if (robot_mode_gyro != mode)
    {
        chassis_yaw_target_mode = mode;
        chassis_yaw_target = 0.0f;
        return 0.0f;
    }

    // 实测相对角：与底盘侧同源（yaw 编码器关节角 − 机械零位偏置）
    const float theta_actual = Lib_Math_WrapAngleNegPIToPI(cmd_gimbal2cmd_data.yaw_motor_position - chassis_gimbal_offset);
    if (robot_mode_gyro != chassis_yaw_target_mode)
        chassis_yaw_target = theta_actual; // 进入 gyro：从实际角起步，不跳变

    chassis_yaw_target_mode = robot_mode_gyro;
    const float spin = knob_scale(cmd_ctx.chassis_speed_channel, chassis_gyro_rotate_speed_min, chassis_gyro_rotate_speed);
    // dt 上限：任务被长时间打断时不让目标角一下跳过太多（正常 2ms）
    const float dt_use = (dt_s > 0.05f) ? 0.05f : dt_s;
    chassis_yaw_target = Lib_Math_WrapAngleNegPIToPI(chassis_yaw_target - spin * dt_use);
    return chassis_yaw_target;
}

/* 过洞放行：切到 hole 档时先停摩擦轮，等 shoot 回报转速已低于阈值，再把 hole 真正下发给云台。
 * cmd_ctx.mode 是摇杆档位的**期望模式**（立即生效）；这里返回的是下发给云台的**生效模式**：
 * 等待摩擦轮降速期间按 normal 下发，云台保持直立，不会轮子还在高速旋转就收进洞里。
 * gimbal_fall_allowed 做**锁存**：一旦放行就不再收回 —— 转速在阈值附近抖一下不该让云台
 * 反复立/倒；离开 hole 档即复位，下次过洞重新等。 */
static uint8_t gimbal_fall_allowed = 0;
static robot_mode gimbal_mode_effective(void)
{
    if (robot_mode_hole != cmd_ctx.mode)
    {
        gimbal_fall_allowed = 0; // 不在过洞档：复位，下次切 hole 重新等降速
        return cmd_ctx.mode;
    }
    if ((0 == gimbal_fall_allowed) && (shoot_friction_speed_low_e == cmd_shoot2cmd_data.friction_speed_low))
    {
        gimbal_fall_allowed = 1;
    }
    return gimbal_fall_allowed ? robot_mode_hole : robot_mode_normal;
}

/*======== 发送层 =========*/
/* 视觉符号标定调试通道（直接写 VOFA，CH13~CH25；axis 占 CH1~CH12）：
 *   CH13 视觉原始 pitch_base_relative (deg，视觉约定向上为负)
 *   CH14 视觉原始 target_pitch_base_relative (deg)
 *   CH15 视觉原始 v_pitch (rad/s)
 *   CH16 视觉原始 a_pitch (rad/s²)
 *   CH17 转换后 cmd→pitch_x (rad，内部向上为正)
 *   CH18 转换后 cmd→pitch_v (rad/s)
 *   CH19 转换后 cmd→pitch_a (rad/s²)
 *   CH20 上 pitch 实测位置 (rad)
 *   CH21 视觉原始 yaw (deg)
 *   CH22 视觉原始 v_yaw (rad/s)
 *   CH23 视觉原始 a_yaw (rad/s²)
 *   CH24 视觉帧时间戳 (s，取 100s 模，判断帧是否更新/卡住)
 *   CH25 视觉控制有效标志 (1=本拍来自视觉源，0=遥控/其他)
 * 非视觉源时 CH13~CH16、CH21~CH24 置 0。
 * 判符号：视觉帧是保持的，只取 CH24 变化（或 CH13/CH17 跳变）的采样点做跨帧差分；
 * 内部位置对时间求导与 CH15 同号 → 当前“位置取反、v/a 不取反”正确，反号则 v/a 也要取反。 */
static void send_visual_debug_vofa(uint8_t valid)
{
    VofaSetChannel(13, valid ? vision_recv_data.pitch_base_relative : 0.0f);
    VofaSetChannel(14, valid ? vision_recv_data.target_pitch_base_relative : 0.0f);
    VofaSetChannel(15, valid ? vision_recv_data.v_pitch_base_relative : 0.0f);
    VofaSetChannel(16, valid ? vision_recv_data.a_pitch_base_relative : 0.0f);
    VofaSetChannel(17, cmd_cmd2gimbal_data.pitch_x);
    VofaSetChannel(18, cmd_cmd2gimbal_data.pitch_v);
    VofaSetChannel(19, cmd_cmd2gimbal_data.pitch_a);
    VofaSetChannel(20, cmd_gimbal2cmd_data.pitch_position);
    VofaSetChannel(21, valid ? vision_recv_data.yaw : 0.0f);
    VofaSetChannel(22, valid ? vision_recv_data.v_yaw : 0.0f);
    VofaSetChannel(23, valid ? vision_recv_data.a_yaw : 0.0f);
    VofaSetChannel(24, valid ? (float)(vision_recv_data.time_stamp % 100000u) * 0.001f : 0.0f);
    VofaSetChannel(25, (float)valid);
}

// 给云台（队列）：统一上下文 → 位置/速度/加速度
static void send_gimbal(void)
{
    // 默认：失能（mode 由状态机给，stop 时 motor 侧不动作）+ 设定值清零
    // 过洞档要等摩擦轮降速，故下发的是**生效模式**而不是 cmd_ctx.mode（见 gimbal_mode_effective）
    cmd_cmd2gimbal_data.mode = gimbal_mode_effective();
    cmd_cmd2gimbal_data.pitch_x = 0.0f;
    cmd_cmd2gimbal_data.pitch_v = 0.0f;
    cmd_cmd2gimbal_data.pitch_a = 0.0f;
    cmd_cmd2gimbal_data.yaw_x = 0.0f;
    cmd_cmd2gimbal_data.yaw_v = 0.0f;
    cmd_cmd2gimbal_data.yaw_a = 0.0f;

#if !CMD_DEBUG_ENABLED(CMD_DEBUG_STAGE_gimbal)
    /* 云台未在 CMD_DEBUG_STAGE 中使能：三轴只发 0 力矩 —— mode 恒 stop，app_gimbal 状态机
     * 进失能、三轴 setref 清零（电机保持使能，仍收零力矩帧）。规划器与视觉分支一并旁路：
     * 云台不动作，累加器也不必被推进。仍记一次控制源，恢复使能后首个交还拍不会误判边沿。 */
    cmd_cmd2gimbal_data.mode = robot_mode_stop;
    cmd_last_control_type = cmd_ctx.type;
    send_visual_debug_vofa(0);
    xQueueOverwrite(cmd2gimbal_queue_handle, &cmd_cmd2gimbal_data);
    return;
#endif

    /* 规划器位置累加器的播种条件（planner 内部为开环累加，不读反馈，故需显式重新锚定）：
     *   ① 失能期间每拍播种：累加器持续跟随反馈，使能首拍目标 = 当前实际位置，不突变。
     *      不用边沿检测是为了不依赖恰好抓到使能那一帧，sbus 掉线恢复也自然覆盖。
     *   ② 视觉交还通道源的首拍播种一次：视觉期间 planner 被绕过、累加器停在旧值，
     *      交还时用当前反馈重新锚定。
     * 视觉接管瞬间（通道源→视觉）不播种：本拍 planner 不被调用，旧值无人消费。 */
    uint8_t seed_position = (robot_mode_stop == cmd_ctx.mode) || ((visual_control_e == cmd_last_control_type) && (visual_control_e != cmd_ctx.type));

    if (visual_control_e == cmd_ctx.type)
    {
        // 视觉源：直接给 位置/速度/加速度，不过规划器
        // 单位约定：视觉 pitch/yaw 为 deg（转 rad）；v/a 为 rad/s、rad/s²（不转）
        // 坐标系：pitch 为 base-relative（与 app_gimbal 一致）；yaw 为世界系（需标定对齐电机系）
        /* 符号：视觉线协议的 pitch **位置** 负方向朝上，本板内部 pitch 正方向朝上 → 位置取反。
         * 速度/加速度 **不取反**：实测视觉发送的 v/a 与位置字段不是同一镜像 —— 用
         * ∫ref_vel·dt 对 (ref - 零位) 做回归，斜率应为 +1 才与位置导数同相，而 v/a 原样
         * 传入时才是同相（取反后反而与位置环反相，前馈拖后腿，小幅视觉目标几乎跟不动）。
         * 零位：视觉以“上 pitch 水平”为 0，而内部 pitch_position 在同一姿态下是
         * VISION_PITCH_ZERO_OFFSET（非 0），故位置还要加回该偏置；速度/加速度是导数，
         * 不含零位。 */
        /* 限幅：视觉是外环直通，异常/突变值会直接变成大误差和大前馈；这里先把视觉角夹到与
         * 规划器相同的物理约束内（数值同源，宏独立命名，见文件头 VISION_*_LIMIT），再加零位偏置。
         * yaw 位置不限幅 —— 它是无限旋转轴，±π 环绕由 yaw 轴误差归一化处理。 */
        cmd_cmd2gimbal_data.pitch_x =
            Lib_Math_Clamp(-DEG_TO_RAD(vision_recv_data.pitch_base_relative), VISION_PITCH_POS_LIMIT_MIN, VISION_PITCH_POS_LIMIT_MAX) +
            VISION_PITCH_ZERO_OFFSET;
        cmd_cmd2gimbal_data.pitch_v = Lib_Math_Clamp(vision_recv_data.v_pitch_base_relative, -VISION_PITCH_SPEED_LIMIT, VISION_PITCH_SPEED_LIMIT);
        cmd_cmd2gimbal_data.pitch_a = Lib_Math_Clamp(vision_recv_data.a_pitch_base_relative, -VISION_PITCH_ACC_LIMIT, VISION_PITCH_ACC_LIMIT);
        cmd_cmd2gimbal_data.yaw_x = DEG_TO_RAD(vision_recv_data.yaw);
        cmd_cmd2gimbal_data.yaw_v = Lib_Math_Clamp(vision_recv_data.v_yaw, -VISION_YAW_SPEED_LIMIT, VISION_YAW_SPEED_LIMIT);
        cmd_cmd2gimbal_data.yaw_a = Lib_Math_Clamp(vision_recv_data.a_yaw, -VISION_YAW_ACC_LIMIT, VISION_YAW_ACC_LIMIT);
        // 底盘跟随前馈：视觉转云台时底盘也要同步跟上，前馈取**限幅后**的角速度，与云台一致
        cmd_gimbal_yaw_rate_cmd = cmd_cmd2gimbal_data.yaw_v;
    }
    else
    {
        // 通道源（sbus / 图传 / 键鼠）：摇杆 -1~1 → 目标速度 → 规划器
        PlannerInput_s in = {0}; // 值初始化，避免漏赋值字段（尤其新增的 seed）
        PlannerOutput_s out;
        in.seed = seed_position; // 两个轴同拍播种

        // pitch：限幅模式（有机械限位）。目标位置由 planner 内部开环累加，下面两个
        // current_* 只在 seed 拍被用来重新锚定累加器（电流外推不再读反馈位置）
        if (robot_mode_hole == cmd_ctx.mode)
        {
            /* 过洞档：**不对 pitch 通道累加**，并把累加器直接钉在 0 = 上 pitch 水平
             * （规划器坐标以 pitchup_position_2「下 pitch 直立、上 pitch 水平」为原点，
             * 限位就是 pitchup_position_1/3 - pitchup_position_2）。
             * 若折叠期间仍按摇杆积分，残留的"低头"目标会在下一次立起时被直接执行 ——
             * 上升阶段上 pitch 跟随 cmd 设定值，就会往车体里扎。钉 0 后每次直立都从
             * 水平开始，立起来以后再按摇杆累加。 */
            pitch_planner.ref_position = 0.0f;
            pitch_planner.position_valid = 1; // 已播种，恢复直立时不再用实测位置重锚
            cmd_cmd2gimbal_data.pitch_x = 0.0f;
            cmd_cmd2gimbal_data.pitch_v = 0.0f;
            cmd_cmd2gimbal_data.pitch_a = 0.0f;
        }
        else
        {
            in.current_position = cmd_gimbal2cmd_data.pitch_position;
            in.current_speed = cmd_gimbal2cmd_data.pitch_vel;
            in.current_acceleration = 0.0f; // 电机无加速度反馈
            in.target_cmd = channel_deadzone(cmd_ctx.gimbal_pitch_channel);
            PlannerCalculate(&pitch_planner, &in, &out);
            cmd_cmd2gimbal_data.pitch_x = out.position;
            cmd_cmd2gimbal_data.pitch_v = out.speed;
            cmd_cmd2gimbal_data.pitch_a = out.acceleration;
        }

        // yaw：环绕模式（无限旋转）。累加器以 IMU 世界系航向为锚点（播种拍），与云台侧 yaw 轴的
        // 反馈同源，所以累加出的目标直接就是世界系角度，不需要在两边做坐标系换算。
        // 开环累加下这一路天然就是"锁世界系航向"：底盘被推动/自转不会改变目标，云台反向补偿
        in.current_position = cmd_gimbal2cmd_data.yaw_position;
        in.current_speed = cmd_gimbal2cmd_data.yaw_vel;
        in.current_acceleration = 0.0f;
        // 取反：yaw 已约定逆时针为正（gimbal 端电机方向镜像），此处补偿以保持摇杆物理转向不变
        in.target_cmd = channel_deadzone(-cmd_ctx.gimbal_yaw_channel);
        PlannerCalculate(&yaw_planner, &in, &out);
        cmd_cmd2gimbal_data.yaw_x = out.position;
        cmd_cmd2gimbal_data.yaw_v = out.speed;
        cmd_cmd2gimbal_data.yaw_a = out.acceleration;
        // 底盘跟随前馈：取规划器输出的轨迹速度（而不是摇杆原值），底盘与云台走同一条
        // 加减速曲线，起步/收手时也不会一个已减速、一个还在冲
        cmd_gimbal_yaw_rate_cmd = out.speed;
    }

    cmd_last_control_type = cmd_ctx.type; // 记录本拍控制源（须在读取 seed_position 之后）

    send_visual_debug_vofa((uint8_t)(visual_control_e == cmd_ctx.type));

    xQueueOverwrite(cmd2gimbal_queue_handle, &cmd_cmd2gimbal_data);
}

/* 给底盘（CAN，每周期一帧；接收已由 CAN 中断写入 chassis2gimbal_data）
 *
 * 摇杆向量定义在"云台指向"坐标系里：推前 = 朝云台指向的方向平移（硬跟随）。
 * 底盘只认车身系的 (vx, vy)，故按云台相对底盘的夹角 θ 旋转过去：v_body = R(θ)·v_stick。
 * 三个模式都旋：gyro 档底盘在自转，若用车身系向量，推"前"会变成绕圈跑；
 * 旋转后不管车身转到哪个角度，推前都是朝准星方向直线平移。
 * 正常模式下车身本来就在被 w 拉向云台，θ 只是跟随滞后剩下的那点小角度，
 * 旋转量很小；等 θ 收敛到 0 就与"直接发车身系 vx/vy"等价。
 * 云台未使能（CMD_DEBUG_STAGE 无 gimbal 位）时走底盘直通：不做 θ 旋转，
 * w 也不由状态机解 —— 改成由空出来的云台 yaw 摇杆通道直接控，见下面的分支。
 *
 * dt：gyro 档目标偏角的积分步长（s），由 AppCmdRun 传入。 */
static void send_chassis(float dt_s)
{
    (void)dt_s; // 底盘未使能时整条解算被编译掉，这里避免 unused 警告
#if !CMD_DEBUG_ENABLED(CMD_DEBUG_STAGE_chassis)
    /* 底盘未在 CMD_DEBUG_STAGE 中使能：使能与三轴速度全部给 0，其余解算旁路。
     * 仍照常发送一帧，底盘侧收到的是"失能 + 零速"。 */
    gimbal2chassis_data.enabled = robot_mode_stop;
    gimbal2chassis_data.power_boost_enable = 0;
    gimbal2chassis_data.vx = 0.0f;
    gimbal2chassis_data.vy = 0.0f;
    gimbal2chassis_data.w = 0.0f;
    gimbal2chassis_data.yaw = 0.0f;
    /* 目标角积分器回零：重新使能后进入 gyro 会重新用实测角播种 */
    chassis_yaw_target = 0.0f;
    chassis_yaw_target_mode = robot_mode_stop;
#else
    /* 平动最大速度：速度旋钮（ch8）已改作底盘驱动档，这里固定取中位值 0，
     * 即 knob_scale(0) = chassis_translate_speed_min 与 ..._speed 的区间中点。
     * 摇杆只决定方向与在该上限内的比例，满舵 = 该固定档位的最大速度。
     * gyro 档的自转角速度用同一个固定值缩放（见 chassis_w_from_mode）。 */
    float translate_speed = knob_scale(cmd_ctx.chassis_speed_channel, chassis_translate_speed_min, chassis_translate_speed);
    // 摇杆速度向量 (vx 前+、vy 左+)：正常路径下先按"云台指向坐标系"取，再由下面的 θ 转到车身系
    float vx_stick = channel_deadzone(cmd_ctx.chassis_vx_channel) * translate_speed;
    float vy_stick = -channel_deadzone(cmd_ctx.chassis_vy_channel) * translate_speed;

    /* 额外功率（ch8 第 1 段）随控制帧转发；底盘侧用它决定功率上限是否加成 */
    gimbal2chassis_data.power_boost_enable = cmd_ctx.power_boost_enable;

    if ((robot_mode_stop == cmd_ctx.mode) || (chassis_drive_disable_e == cmd_ctx.chassis_drive))
    {
        /* 总开关关闭 / ch8 第 4 段：底盘失能（只影响底盘，云台照常工作）。
         * 指令全给 0；目标角积分器回零，重新使能后 gyro 会重新用实测角播种。 */
        gimbal2chassis_data.enabled = robot_mode_stop;
        gimbal2chassis_data.vx = 0.0f;
        gimbal2chassis_data.vy = 0.0f;
        gimbal2chassis_data.w = 0.0f;
        gimbal2chassis_data.yaw = 0.0f;
        chassis_yaw_target = 0.0f;
        chassis_yaw_target_mode = robot_mode_stop;
    }
    else if (chassis_drive_free_e == cmd_ctx.chassis_drive)
    {
        /* ch8 第 3 段：自由模式 —— 底盘照常 RUN，但不跟随云台、不转车身：
         *   底盘侧收到 robot_mode_free，跟随环直接输出 0（见 ChassisWCommand）；
         *   vx/vy 按**车身系**直通（不按云台夹角旋转），推前 = 车头方向。 */
        gimbal2chassis_data.enabled = robot_mode_free;
        gimbal2chassis_data.vx = vx_stick;
        gimbal2chassis_data.vy = vy_stick;
        gimbal2chassis_data.w = 0.0f;
        gimbal2chassis_data.yaw = 0.0f; // free 档底盘不读目标角
        /* 目标角积分器回零：切回 normal/gyro 时不带自由模式的旧值 */
        chassis_yaw_target = 0.0f;
        chassis_yaw_target_mode = robot_mode_stop;
    }
    else
    {
        gimbal2chassis_data.enabled = cmd_ctx.mode; // robot_mode_stop 已在上面处理

#if CMD_DEBUG_STAGE_CHASSIS_DIRECT
        /* 云台未使能：摇杆向量不按云台夹角旋转，直接按车身系发下去 ——
         * 推前就是车头方向，底盘不再追云台。
         * w 改由**云台 yaw 那个摇杆通道**直接控制（这个模式下它空着，正好拿来转车身）：
         *   摇杆 -1~1 → 自转角速度，大小仍按 gyro 档那套固定中位值缩放；
         *   取负与云台 yaw 的约定保持一致：正摇杆 = 云台顺时针，底盘也该顺时针，
         *   而 w>0 是逆时针 ⇒ 这里必须取负（与 normal 档 ff=+1 的结果同向）；
         *   死区照吃，摇杆回中 = 不转。
         * 这个模式没有云台跟随：档位报成 gyro，让底盘直接执行 w；目标偏角填当前
         * 实测角，跟随误差恒为 0，不会额外叠加 PID 输出（stop 仍报 stop）。 */
        gimbal2chassis_data.enabled = (robot_mode_stop == cmd_ctx.mode) ? robot_mode_stop : robot_mode_gyro;
        gimbal2chassis_data.vx = vx_stick;
        gimbal2chassis_data.vy = vy_stick;
        gimbal2chassis_data.w =
            -channel_deadzone(cmd_ctx.gimbal_yaw_channel) * knob_scale(cmd_ctx.chassis_speed_channel, chassis_gyro_rotate_speed_min, chassis_gyro_rotate_speed);
        /* 目标角 = 实测角：底盘侧 e = wrap(θ_实测 − θ_ref) ≈ 0。
         * 云台失能时编码器仍在回传 yaw_motor_position，故这里取当前值即可。 */
        gimbal2chassis_data.yaw = Lib_Math_WrapAngleNegPIToPI(cmd_gimbal2cmd_data.yaw_motor_position - chassis_gimbal_offset);
        chassis_yaw_target = 0.0f;
        chassis_yaw_target_mode = robot_mode_stop;
#else
        /* 云台指向相对底盘前进方向的真实夹角 θ (rad)，逆时针为正。
         * 取 yaw 电机编码器关节角（云台相对底盘）再减掉机械零位偏置：装配后编码器零位
         * 与"云台指向 = 底盘前进方向"对不齐，差的就是 chassis_gimbal_offset
         * （底盘前进方向本身已由底盘侧的舵轮零位标定好，云台这边只补这一个角）。
         * 摇杆向量的旋转和底盘跟随都必须用这个 θ：一个决定往哪走，一个决定车身转到哪，
         * 两者不同源就会差一个偏置角（车身停在编码器零位，向量却按真实朝向旋转）。
         * 用的是 yaw_motor_position（编码器关节角）而不是 yaw_position —— 后者是 IMU 的
         * 世界系航向，底盘一转就变，拿它做跟随会持续自转。
         *
         * θ 只在本板用于摇杆向量旋转。底盘不接收实测角：它从共享 CAN1 直接读同一颗
         * RS05，自己扣同样的 chassis_gimbal_offset，拿到最新鲜的实测 θ。
         * 本帧的 yaw 字段发的是**目标偏角** θ_ref（normal/hole = 0），
         * 底盘用 wrap(θ_实测 − θ_ref) 作为 PID 输入。
         * 云台只保留需要本板指令角速度的前馈项（见 chassis_w_from_mode）。 */
        float theta = Lib_Math_WrapAngleNegPIToPI(cmd_gimbal2cmd_data.yaw_motor_position - chassis_gimbal_offset);
        /* 摇杆向量专用角：把 θ 外推到「底盘真正执行本拍车体系速度」的时刻。
         * gyro 档云台锁世界系航向、底盘以实测 w 逆时针自转 ⇒ dθ/dt = -w。
         * chassis2gimbal_data.w 是底盘 IMU 实测角速度（app_cmd.c:108 已在接收）。 */
        float theta_vec = Lib_Math_WrapAngleNegPIToPI(theta - chassis_vec_theta_delay_s * chassis2gimbal_data.w);
        float c = Lib_Math_Cos(theta_vec);
        float s = Lib_Math_Sin(theta_vec);
        gimbal2chassis_data.vx = c * vx_stick - s * vy_stick;
        gimbal2chassis_data.vy = s * vx_stick + c * vy_stick;
        // 目标偏角：normal/hole = 0，gyro 按自转速度斜坡（向量旋转仍用上面的 theta）
        gimbal2chassis_data.yaw = chassis_yaw_target_from_mode(dt_s);
        // w 只发前馈/自转部分，PID 由底盘用实测 θ 与 yaw 目标自行计算
        // （前馈同拍由 send_gimbal 写入 cmd_gimbal_yaw_rate_cmd）
        gimbal2chassis_data.w = chassis_w_from_mode(cmd_gimbal_yaw_rate_cmd);
#endif // CMD_DEBUG_STAGE_CHASSIS_DIRECT
    } // else：普通 / 额外功率 / 调试直通
#endif // !CMD_DEBUG_ENABLED(CMD_DEBUG_STAGE_chassis)

    CommSend(&chassis_comm, (uint8_t *)&gimbal2chassis_data);
}

// 给射击（队列）
static void send_shoot(void)
{
#if !CMD_DEBUG_ENABLED(CMD_DEBUG_STAGE_shoot)
    /* 发射未在 CMD_DEBUG_STAGE 中使能：只发 0 力矩 —— friction_mode=disable 让摩擦轮直接
     * 0 电流、拨盘同样 0 电流，fire_mode=off 关掉拨弹状态机。 */
    cmd_cmd2shoot_data.friction_mode = shoot_friction_disable_e;
    cmd_cmd2shoot_data.fire_mode = fire_mode_off_e;
#else
    /* 摩擦轮三段模式的判定（按语义在这里定，shoot 侧只执行）：
     *   - ch4 断开 / 遥控掉线 → disable（掉线时 sbus 数据是陈旧的，按"全部失能"的既有约定处理）；
     *   - ch9 停在"关闭摩擦轮"段 → disable（操作手显式停轮，优先级高于下面所有按姿态的自动判断）；
     *   - 期望档位是 hole → slowdown：先缓慢减速到 0，shoot 回报转速低于阈值后
     *     gimbal_mode_effective 才放行云台倒下。等待期云台仍按 normal 保持在直立位，
     *     若此时用 disable，轮子只靠摩擦自由滑行、降速太慢；用 slowdown 是速度环跟随斜坡。
     *   - 云台直立 且 没有在等过洞 → spin（满速）；其余（失能/立起中/倒下中/倒下）→ disable，
     *     避免云台还没立起来/过洞收着就把弹打出去。只看 gimbal_state_stand_e，不含立起中。 */
    uint8_t remote_on = (sbus_inst.daemon->is_online == 1) && (sbus_inst.sbus_data.ch[SBUS_CH_ENABLE] > sbus_half);
    if (0 == remote_on)
    {
        cmd_cmd2shoot_data.friction_mode = shoot_friction_disable_e;
    }
    else if (cmd_ctx.friction_off)
    {
        cmd_cmd2shoot_data.friction_mode = shoot_friction_disable_e;
    }
    else if (robot_mode_hole == cmd_ctx.mode)
    {
        cmd_cmd2shoot_data.friction_mode = shoot_friction_slowdown_e;
    }
    else if (gimbal_state_stand_e == cmd_gimbal2cmd_data.state)
    {
        cmd_cmd2shoot_data.friction_mode = shoot_friction_spin_e;
    }
    else
    {
        cmd_cmd2shoot_data.friction_mode = shoot_friction_disable_e;
    }
    cmd_cmd2shoot_data.fire_mode = cmd_ctx.fire_mode; // ch7 电平已内含：off = ch7 断开 / 关闭摩擦轮段 / ch4 关/掉线
#endif
    /* 射频：视觉接管时由 enable_pitch_diff 换算（见 input_update），其余源为 0。
     * 两个裁判校准量暂无来源（裁判消息链路还没接）→ 保持 0。 */
    cmd_cmd2shoot_data.fire_rate = cmd_ctx.fire_rate;
    xQueueOverwrite(cmd2shoot_queue_handle, &cmd_cmd2shoot_data);
}

// 给视觉（USB 虚拟串口）
static void send_vision(void)
{
    vision_send_data.cmd_ID = VISUAL_CMD_TX;
    vision_send_data.time_stamp = (uint32_t)(DWT_GetTimeUs() / 1000); /* 板卡时间戳 ms */
    if (visual_control_e == cmd_ctx.type)
    {
        vision_send_data.mode = vision_mode_auto_aim_e;
    }
    else
    {
        vision_send_data.mode = vision_mode_idle_e;
    }
    // yaw/yaw_vel 回传的是 IMU 世界系量，与协议约定的"世界系 yaw"一致
    vision_send_data.yaw = RAD_TO_DEG(cmd_gimbal2cmd_data.yaw_position);
    /* pitch 与接收帧同一符号约定：线上 **负方向朝上**，本板内部正方向朝上 → 回传取反
     * （位置/速度一致）。位置还要先减掉内部零位 VISION_PITCH_ZERO_OFFSET，换成“水平=0”
     * 的视觉坐标后再取反。pitch_down 是下 pitch 机构位姿角，另一物理量，不参与镜像。 */
    vision_send_data.pitch_base_relative = RAD_TO_DEG(-(cmd_gimbal2cmd_data.pitch_position - VISION_PITCH_ZERO_OFFSET));
    vision_send_data.pitch_down = RAD_TO_DEG(cmd_gimbal2cmd_data.pitch_down_position); // 下pitch位姿角 (rad→deg)
    vision_send_data.yaw_vel = cmd_gimbal2cmd_data.yaw_vel;
    vision_send_data.pitch_base_relative_vel = -cmd_gimbal2cmd_data.pitch_vel;
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
        // 通道原始值范围（SBUS 协议标准值，换遥控器/重新校准时改这里）
        .ch_range =
            {
                .ch_min = FS_SBUS_CH_MIN,
                .ch_max = FS_SBUS_CH_MAX,
                .ch_center = FS_SBUS_CH_CENTER,
            },
    };
    BSP_ASSERT_APP_CALL(SBUSConfig(&sbus_inst, &sbus_cfg));

    // 初始化规划器（位置限幅/位置模式/最大速度/最大加速度）
    /* TODO 实车核对 pitch 限位基准：这里用的是「下 pitch 直立、上 pitch 水平」为原点的量纲
     *      （U1-U2 / U3-U2），而 gimbal 回传的 pitch_position 是以「两轴都倒下」为原点的
     *      (u - U0) - (d - Dmin)（app_gimbal.c 的反馈合成式）。两者相差约 0.0213 rad(≈1.2°)：
     *      当前下限比机械可达下限低 1.2°、上限提前 1.2° 停住。开环累加后目标不再每拍被反馈
     *      拉回，停在错位边界上的表现会更明显。确认标定姿态后应改为
     *      U1-U0-(Dmax-Dmin) / U3-U0-(Dmax-Dmin)（≈ -0.4359 / 0.7876）。 */
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
        .daemon_fault = DAEMON_FAULT_NONE,
        .daemon_reload = 10,
    };
    BSP_ASSERT_APP_CALL(CommRegister(&vis_comm));
    BSP_ASSERT_APP_CALL(CommConfig(&vis_comm, &vis_cfg));

    // 底盘通信（CAN1 / IDSEQ / CUSTOM）：登记 + 配置介质（ID 段/帧格式）+ 链路看门狗。
    // 一个实例双向：云台在 tx_id 段发、rx_id 段收（idseq 后端已支持收发异段）。
    CommMediaCanIdseqConfig_s chassis_comm_media_cfg = {
        .can_e = CAN_1,
        .tx_id = 0x100, /* 云台→底盘 ID 段基址 */
        .rx_id = 0x110, /* 底盘→云台 ID 段基址 */
        .frame_type = CAN_STANDARD_DATA_FRAME,
        .mode = CAN_FRAME_FORMAT_CLASSIC, /* CAN1 上有经典 CAN 的 RS05，不可用 FD */
        .timeout_ms = 1,
    };
    CommConfig_s chassis_comm_cfg = {
        .media_cfg = &chassis_comm_media_cfg, /* CAN 后端必须有介质配置（ID 段在此写入） */
        .on_frame = ChassisRecvOnFrame,
        .daemon_fault = DAEMON_FAULT_NONE,
        .daemon_reload = 10, /* 对端每 2ms 发一帧，超 10ms 判离线 */
    };
    BSP_ASSERT_APP_CALL(CommRegister(&chassis_comm));
    BSP_ASSERT_APP_CALL(CommConfig(&chassis_comm, &chassis_comm_cfg));
}

ITCM_RAM void AppCmdRun(float dt, uint64_t time_stamp)
{
    (void)time_stamp;
    // 1. 收集反馈
    xQueueReceive(gimbal2cmd_queue_handle, &cmd_gimbal2cmd_data, 0);
    xQueueReceive(shoot2cmd_queue_handle, &cmd_shoot2cmd_data, 0);

    // 2. 输入：选源 + 把选中源的输入翻译成统一 4 通道
    input_update();

    // 3. 统一上下文 → 各执行器设定值并发送
    send_gimbal(); // 给云台
    /* APP_TASK_DEF 的 dt 是 ms；send_chassis 内部按秒积分 gyro 目标角。
     * ⚠ 这里必须换算：直接传 ms 会让 θ_ref 以 25 倍速度旋转、每 ~39ms 绕一圈，
     *   底盘 yaw PID 追不上就形成 ~25Hz 极限环，经逆解把舵角目标打得来回跳。 */
    send_chassis(dt * 0.001f);
    send_shoot();  // 给射击
    send_vision(); // 给视觉
}
