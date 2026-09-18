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
#include "comm_media_can_idseq.h"
#include "comm_proto_custom.h"
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
         （已实现：机器人侧宏 chassis_follow_ff + chassis_w_from_mode 的前馈项）
gimbal：底盘旋转导致云台底座旋转，需要前馈补偿，不然等出现误差再纠正就延迟了。（先不加，做todo）
 */

/*============================================
 *              宏
 *============================================*/
// 摇杆死区：通道值小于该值视为 0，避免中心抖动引起缓慢漂移
#define DEADZONE (0.01f)
#define sbus_half 0.5 // 判断开关通道float等于1/-1/0

/* SBUS 通道分配（0 基；摇杆 -1~1，三档开关 -1/0/+1，旋钮 -1~1）
 * 注：README「操作逻辑」一节写的拨杆号与代码不一致（那里把自瞄放在第 7 路、
 *     第 5 路未用），这里以实机在用的映射为准。 */
#define SBUS_CH_VY 0     // 右摇杆左右 → 底盘 vy
#define SBUS_CH_VX 1     // 右摇杆前后 → 底盘 vx
#define SBUS_CH_PITCH 2  // 左摇杆上下 → 云台 pitch
#define SBUS_CH_YAW 3    // 左摇杆左右 → 云台 yaw
#define SBUS_CH_ENABLE 4 // 总开关：上 = 使能
#define SBUS_CH_AIM 5    // 自瞄开关：与视觉"有目标"同时成立才把云台交给视觉
#define SBUS_CH_MODE 6   // 档位开关（三档）：normal / gyro / hole
#define SBUS_CH_FIRE 7   // 开火
#define SBUS_CH_SPEED 8  // 平动速度旋钮：缩放底盘平动最大速度（最小档 ~ 最大档）
#define SBUS_CH_ROTATE 9 // 旋转速度旋钮：缩放 gyro(小陀螺)档的自转角速度（最小档 ~ 最大档）

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

/* AppCmdRun 的帧内上下文：{sbus遥控，图传，键鼠，视觉} 统一接口控制 {云台，底盘，发射，视觉}。
 *
 * 输入源（sbus / 图传 / 键鼠）只负责把自身输入翻译成这里的 归一化通道(-1~1)，
 * 外加 状态机/开火 两个开关量；下游（云台规划、底盘解算、开火、状态回传）只认这份
 * 上下文，不再各自去读某个遥控源。视觉源是特例：不写通道，直接给出云台 位置/速度/加速度。 */
typedef struct
{
    cmd_control_type type; // 本帧生效的控制源

    robot_mode mode; // 状态机：stop = 失能，normal/gyro/hole 由档位开关解出
    uint8_t fire;    // 开火

    // 归一化通道 (-1~1)：摇杆/开关中位 0
    float gimbal_pitch_channel;   // 云台 pitch
    float gimbal_yaw_channel;     // 云台 yaw
    float chassis_vx_channel;     // 底盘前后
    float chassis_vy_channel;     // 底盘左右
    float chassis_speed_channel;  // 底盘平动速度旋钮：-1 = 最小速度档，+1 = 最大速度档（非回中，无死区）
    float chassis_rotate_channel; // 小陀螺转速旋钮：同上，缩放 gyro 档自转角速度
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
/* 底盘通信对话：CAN1 / IDSEQ / PROTO_CUSTOM（帧 = [0xA5][seq][payload N][CRC8][0x5A]）。
 * 云台侧 tx_id=0x100 段、rx_id=0x110 段（底盘侧对调）；ID 段与 CAN1 现有占用
 * （yaw RS05 0x001/0x0FD）及后续 3508 波盘（0x1FF/0x200/0x201~0x208）均不重叠。
 * 收发 payload 分别为 gimbal2chassis_data_t(13B) / chassis2gimbal_data_t(12B)；
 * CAN1 上挂有经典 CAN 的 RS05，故 mode 必须 CLASSIC（不可用 FD）。 */
COMM_DEF(chassis_comm, MEDIA_CAN_IDSEQ, CUSTOM, CUSTOM, chassis2gimbal_data_t, 12, gimbal2chassis_data_t, 13, UNPACK_IN_ISR);

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
static float knob_scale(float ch, float v_min, float v_max)
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
    // 旋钮不回中、也不该被死区吃掉，原值直接用
    cmd_ctx.chassis_speed_channel = sbus_inst.sbus_data.ch[SBUS_CH_SPEED];
    cmd_ctx.chassis_rotate_channel = sbus_inst.sbus_data.ch[SBUS_CH_ROTATE];
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
    cmd_ctx.gimbal_pitch_channel = 0.0f;
    cmd_ctx.gimbal_yaw_channel = 0.0f;
    cmd_ctx.chassis_vx_channel = 0.0f;
    cmd_ctx.chassis_vy_channel = 0.0f;
    cmd_ctx.chassis_speed_channel = 0.0f;  // 中位 = 速度区间中点（失能时无意义）
    cmd_ctx.chassis_rotate_channel = 0.0f; // 中位 = 转速区间中点（失能时无意义）

    /* 源选择暂时只建立在 SBUS 遥控在线的基础上：遥控掉线 → 全部失能。
     * 图传/键鼠接入后，在这里追加各自的在线判断与优先级。 */
    if (sbus_inst.daemon->is_online != 1)
        return;
    if (sbus_inst.sbus_data.ch[SBUS_CH_ENABLE] <= sbus_half)
        return; // 总开关断开 → 失能停车

    // 总开关已打开：解出状态机与开火
    // 档位开关（三档）→ 状态机。沿用历史实现的极性：-1 = normal（上档）、
    // 0 = gyro（中档）、+1 = hole（下档），与 README「从上到下 0/1/2」顺序一致；
    // 若实车拨杆方向相反，交换首尾两个判断即可。
    float sw = sbus_inst.sbus_data.ch[SBUS_CH_MODE];
    if (sw < -sbus_half)
        cmd_ctx.mode = robot_mode_normal; // 上档
    else if (sw > sbus_half)
        cmd_ctx.mode = robot_mode_hole; // 下档
    else
        cmd_ctx.mode = robot_mode_gyro; // 中档
    cmd_ctx.fire = (sbus_inst.sbus_data.ch[SBUS_CH_FIRE] > sbus_half) ? 1 : 0;

    // 云台控制权：自瞄开关打开且视觉有目标 → 交给视觉，否则由摇杆控制
    if ((sbus_inst.sbus_data.ch[SBUS_CH_AIM] > sbus_half) && (vision_appear_e == vision_recv_data.appear))
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
}

/*======== 处理层：统一上下文 → 各执行器设定值 =========*/
/* 底盘自转速度 w (rad/s)：底盘是云台的纯伺服机构，只执行 (enabled, vx, vy, w)，
 * 模式相关的 w 全部在这里按状态机算好再发下去。
 *   normal / hole：跟随——把"云台相对底盘的角度"θ 拉回 0，底盘即转回云台指向
 *   gyro         ：定值自转（底盘自转、云台锁世界系航向，操作手照常瞄准），转速由 ch9 旋钮缩放
 *   stop         ：0
 *
 * θ 是"云台相对底盘的夹角、逆时针为正"，由调用方算出后传入（见 send_chassis）。
 * 两边角度约定一致：yaw 电机 feedback_direction 已镜像成"逆时针为正"，
 * 底盘侧 w 的定义也是"逆时针为正"（见 app_chassis 的 HalfRudderInverse 注释）。
 *
 * 跟随律是"比例项 + 前馈项"：w = chassis_follow_kp × wrap(θ) + chassis_follow_ff × ω_yaw。
 * 比例项：底盘朝让 θ 归零的方向转（θ = 0 即云台指向与底盘前进方向重合），
 * 误差按 kp 的速率指数收敛。该律成立的前提是云台轴锁"世界系航向"（IMU 反馈）：
 * 底盘转动时云台的世界指向不动，θ 才会真的变化。若云台轴锁的是关节角，
 * 底盘一转云台就被拖着同转、θ 恒定，跟随环没有负反馈，w 会一直停在初值上自转。
 * 前馈项（ω_yaw = 云台 yaw 轴的指令角速度，由调用方传入）：比例项只能等 θ 建立误差后
 * 才动作，操作手推 yaw 摇杆时底盘起步慢半拍；前馈让底盘与云台"同速转"，
 * θ 根本不被拉开，kp 只收拾残余误差（详见 robot_def.h 的注释）。
 *
 * ⚠ 实车标定：先架空轮子给一个小 yaw 偏角，确认底盘转向能让 θ 归零；
 *   转向反了就把 kp 取负（本车实际符号与上面"逆时针为正"的推导相反，kp 与 ff 都取了负）。
 *   再把 |kp| 从 1 左右往上加到跟得上又不抖。 */
static float chassis_w_from_mode(float theta, float yaw_rate_cmd)
{
    if (robot_mode_gyro == cmd_ctx.mode)
    {
        // gyro 档底盘在定速自转、云台锁世界系航向让操作手照常瞄准：两者互不跟随，
        // 不加 yaw 前馈（否则瞄准时摇杆会叠加到自转速度上，自转不再是定值），
        // 也不吃跟随限幅——这个定值由旋转速度旋钮（ch9）缩放后恒等输出
        return knob_scale(cmd_ctx.chassis_rotate_channel,
                          chassis_gyro_rotate_speed_min, chassis_gyro_rotate_speed);
    }

    if ((robot_mode_normal != cmd_ctx.mode) && (robot_mode_hole != cmd_ctx.mode))
        return 0.0f; // stop：失能，w 给 0

    // 前馈：跟着云台的指令角速度一起转，底盘不等误差、起步即同步。
    // 不受死区影响——云台静止时 ω_yaw 自然是 0，死区只需管住下面那个比例项
    float w = chassis_follow_ff * yaw_rate_cmd;
    // 死区内不跟随：θ 已经在 0 附近，再给比例项只会让车身被编码器噪声/传动回差推着来回蹭
    if (Lib_Math_Fabs(theta) > chassis_follow_deadzone)
        w += chassis_follow_kp * theta;

    // 限幅只压在跟随输出（比例项 + 前馈）上：前馈可能会把 w 顶到云台 yaw_speed 那么大，
    // 超出底盘能跟上的能力，限幅既防打滑也防标定发散的抖动
    return Lib_Math_Clamp(w, -chassis_follow_w_limit, chassis_follow_w_limit);
}

/*======== 发送层 =========*/
// 给云台（队列）：统一上下文 → 位置/速度/加速度
static void send_gimbal(void)
{
    // 默认：失能（mode 由状态机给，stop 时 motor 侧不动作）+ 设定值清零
    cmd_cmd2gimbal_data.mode = cmd_ctx.mode;
    cmd_cmd2gimbal_data.pitch_x = 0.0f;
    cmd_cmd2gimbal_data.pitch_v = 0.0f;
    cmd_cmd2gimbal_data.pitch_a = 0.0f;
    cmd_cmd2gimbal_data.yaw_x = 0.0f;
    cmd_cmd2gimbal_data.yaw_v = 0.0f;
    cmd_cmd2gimbal_data.yaw_a = 0.0f;

    /* 规划器位置累加器的播种条件（planner 内部为开环累加，不读反馈，故需显式重新锚定）：
     *   ① 失能期间每拍播种：累加器持续跟随反馈，使能首拍目标 = 当前实际位置，不突变。
     *      不用边沿检测是为了不依赖恰好抓到使能那一帧，sbus 掉线恢复也自然覆盖。
     *   ② 视觉交还通道源的首拍播种一次：视觉期间 planner 被绕过、累加器停在旧值，
     *      交还时用当前反馈重新锚定。
     * 视觉接管瞬间（通道源→视觉）不播种：本拍 planner 不被调用，旧值无人消费。 */
    uint8_t seed_position =
        (robot_mode_stop == cmd_ctx.mode) ||
        ((visual_control_e == cmd_last_control_type) && (visual_control_e != cmd_ctx.type));

    if (visual_control_e == cmd_ctx.type)
    {
        // 视觉源：直接给 位置/速度/加速度，不过规划器
        // 单位约定：视觉 pitch/yaw 为 deg（转 rad）；v/a 为 rad/s、rad/s²（不转）
        // 坐标系：pitch 为 base-relative（与 app_gimbal 一致）；yaw 为世界系（需标定对齐电机系）
        cmd_cmd2gimbal_data.pitch_x = DEG_TO_RAD(vision_recv_data.pitch_base_relative);
        cmd_cmd2gimbal_data.pitch_v = vision_recv_data.v_pitch_base_relative;
        cmd_cmd2gimbal_data.pitch_a = vision_recv_data.a_pitch_base_relative;
        cmd_cmd2gimbal_data.yaw_x = DEG_TO_RAD(vision_recv_data.yaw);
        cmd_cmd2gimbal_data.yaw_v = vision_recv_data.v_yaw;
        cmd_cmd2gimbal_data.yaw_a = vision_recv_data.a_yaw;
        // 底盘跟随前馈：视觉转云台时底盘也要同步跟上，前馈同样取视觉给的角速度
        cmd_gimbal_yaw_rate_cmd = vision_recv_data.v_yaw;
    }
    else
    {
        // 通道源（sbus / 图传 / 键鼠）：摇杆 -1~1 → 目标速度 → 规划器
        PlannerInput_s in = {0}; // 值初始化，避免漏赋值字段（尤其新增的 seed）
        PlannerOutput_s out;
        in.seed = seed_position; // 两个轴同拍播种

        // pitch：限幅模式（有机械限位）。目标位置由 planner 内部开环累加，下面两个
        // current_* 只在 seed 拍被用来重新锚定累加器（电流外推不再读反馈位置）
        in.current_position = cmd_gimbal2cmd_data.pitch_position;
        in.current_speed = cmd_gimbal2cmd_data.pitch_vel;
        in.current_acceleration = 0.0f; // 电机无加速度反馈
        in.target_cmd = channel_deadzone(cmd_ctx.gimbal_pitch_channel);
        PlannerCalculate(&pitch_planner, &in, &out);
        cmd_cmd2gimbal_data.pitch_x = out.position;
        cmd_cmd2gimbal_data.pitch_v = out.speed;
        cmd_cmd2gimbal_data.pitch_a = out.acceleration;

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

    xQueueOverwrite(cmd2gimbal_queue_handle, &cmd_cmd2gimbal_data);
}

/* 给底盘（CAN，每周期一帧；接收已由 CAN 中断写入 chassis2gimbal_data）
 *
 * 摇杆向量定义在"云台指向"坐标系里：推前 = 朝云台指向的方向平移（硬跟随）。
 * 底盘只认车身系的 (vx, vy)，故按云台相对底盘的夹角 θ 旋转过去：v_body = R(θ)·v_stick。
 * 三个模式都旋：gyro 档底盘在自转，若用车身系向量，推"前"会变成绕圈跑；
 * 旋转后不管车身转到哪个角度，推前都是朝准星方向直线平移。
 * 正常模式下车身本来就在被 w 拉向云台，θ 只是跟随滞后剩下的那点小角度，
 * 旋转量很小；等 θ 收敛到 0 就与"直接发车身系 vx/vy"等价。 */
static void send_chassis(void)
{
    /* 云台指向相对底盘前进方向的真实夹角 θ (rad)，逆时针为正。
     * 取 yaw 电机编码器关节角（云台相对底盘）再减掉机械零位偏置：装配后编码器零位
     * 与"云台指向 = 底盘前进方向"对不齐，差的就是 chassis_gimbal_offset
     * （底盘前进方向本身已由底盘侧的舵轮零位标定好，云台这边只补这一个角）。
     * 底盘跟随的 w 和摇杆向量的旋转都必须用这个 θ：一个决定车身转到哪，一个决定往哪走，
     * 两者不同源就会差一个偏置角（车身停在编码器零位，向量却按真实朝向旋转）。
     * 用的是 yaw_motor_position（编码器关节角）而不是 yaw_position —— 后者是 IMU 的
     * 世界系航向，底盘一转就变，拿它做跟随会持续自转。 */
    float theta = Lib_Math_WrapAngleNegPIToPI(cmd_gimbal2cmd_data.yaw_motor_position - chassis_gimbal_offset);
    float c = Lib_Math_Cos(theta);
    float s = Lib_Math_Sin(theta);
    /* 平动速度旋钮调速：ch8 (-1~1) 映射成本拍可用的平动最大速度
     *   knob = -1 → chassis_translate_speed_min（慢速档）
     *   knob = +1 → chassis_translate_speed（全速档）
     * 摇杆只决定方向与在该上限内的比例，满舵 = 该档位的最大速度。 */
    float translate_speed = knob_scale(cmd_ctx.chassis_speed_channel,
                                       chassis_translate_speed_min, chassis_translate_speed);
    // 摇杆 → 云台指向坐标系下的速度向量 (vx 前+、vy 左+)
    float vx_stick = channel_deadzone(cmd_ctx.chassis_vx_channel) * translate_speed;
    float vy_stick = -channel_deadzone(cmd_ctx.chassis_vy_channel) * translate_speed;

    gimbal2chassis_data.enabled = cmd_ctx.mode; // robot_mode_stop = 失能
    gimbal2chassis_data.vx = c * vx_stick - s * vy_stick;
    gimbal2chassis_data.vy = s * vx_stick + c * vy_stick;
    // w 含云台指令角速度前馈（同拍由 send_gimbal 写入 cmd_gimbal_yaw_rate_cmd）
    gimbal2chassis_data.w = chassis_w_from_mode(theta, cmd_gimbal_yaw_rate_cmd);

    CommSend(&chassis_comm, (uint8_t *)&gimbal2chassis_data);
}

// 给射击（队列）
static void send_shoot(void)
{
    cmd_cmd2shoot_data.fire_or_not = cmd_ctx.fire;
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
        .ch_range = {
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

ITCM_RAM void AppCmdRun(void)
{
    // 1. 收集反馈
    xQueueReceive(gimbal2cmd_queue_handle, &cmd_gimbal2cmd_data, 0);
    xQueueReceive(shoot2cmd_queue_handle, &cmd_shoot2cmd_data, 0);

    // 2. 输入：选源 + 把选中源的输入翻译成统一 4 通道
    input_update();

    // 3. 统一上下文 → 各执行器设定值并发送
    send_gimbal();  // 给云台
    send_chassis(); // 给底盘
    send_shoot();   // 给射击
    send_vision();  // 给视觉
}
