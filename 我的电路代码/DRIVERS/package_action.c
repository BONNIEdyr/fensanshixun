#include "package_action.h"
#include "servo_pwm.h"
#include "Emm_V5.h"
#include "config.h"
#include "delay.h"

/* 滑轨命令串口保护间隔(ms)：USART1 由滑轨与4个轮边电机共用，驱动器靠串口空闲
 * (IDLE)分帧。下发滑轨命令前后各留一段静默期，确保其与轮边命令不会在总线上
 * 粘连成一帧而被驱动器校验失败丢弃。方案B：边移动边取放料时避免滑轨丢命令。 */
#define SLIDE_CMD_GUARD_MS   5


/* ============================================================
 *  包执行器状态机
 * ============================================================ */
typedef enum {
    PACK_STATE_IDLE,            // 空闲
    PACK_STATE_WAIT_SLIDE,      // 等待滑轨就绪
    PACK_STATE_DELAY,           // 等待延时
} PackState_t;

/* 执行器内部状态 */
static struct {
    PackState_t  state;                    // 状态机状态
    uint8_t      currentPkgIdx;            // 当前执行到第几个包 (0~5)
    uint8_t      currentStepIdx;           // 当前执行到包内的第几步
    uint8_t      completedTriggerCount;    // 当前包已经完成的按键触发次数
    uint16_t     waitTimer;                // 延时/等待计时器 (tick)
    uint8_t      busy;                     // 忙标志
    uint8_t      slideTriggered;           // 本步是否已触发了滑轨运动
} g_pkg;

#define TRAY_POS_DYNAMIC 0xFFFE

/* ============================================================
 *  包动作步骤定义
 *  本文件包含若干包的步骤数组，当前离散调用表仅使用6个具体包：
 *    LOAD_TRAY_1/2/3 与 UNLOAD_TRAY_1/2/3
 *  注释格式：
 *    // N. 动作描述
 *  滑轨4个高度：过渡位13cm / 托盘放置位10cm / 物料夹取位4.2cm / 物料放置位0cm（零位）
 *  夹爪：CLAW_POS_GRIP=0°闭合夹取 / CLAW_POS_RELEASE=60°张开放料
 *  云台：PTZ_POS_GRIP=0°夹取位 / PTZ_POS_TRAY1/2/3=托盘位1/2/3
 *  托盘：TRAY_POS_1/2/3/4=托盘1/2/3/4号位
 *  滑轨绝对位置宏（脉冲数）：
 *    SLIDE_POS_TRANSIT       = 10400（过渡位13cm）
 *    SLIDE_POS_TRAY_PLACE    =  8000（托盘放置位10cm）
 *    SLIDE_POS_GRAB          =  3360（物料夹取位4.2cm）
 *    SLIDE_POS_PLACE         =     0（物料放置位0cm，零位）
 *   所有滑轨控制使用绝对位置模式 (raF=1)
 * ============================================================ */

/* ---------- LOAD_TRAY_3：夹取物料 → 云台转到托盘3号位 → 放到托盘3号位 ---------- */
static const PackStep_t g_steps_pkg0[] = { 
    // 1. 滑轨从过渡位下降到物料夹取位（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_GRAB,              60},
    // 2. 夹爪从张开位(60°)回到零位(0°)夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨从夹取位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转到托盘位3，只动云台舵机，别的舵机不动（270°舵机大幅转动，需更长等待）
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY3, 0xFFFF,      0,                           45},
    // 5. 滑轨从过渡位下降到物料托盘放置位（绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},

    // 6. 夹爪从零位回到张开位放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 7. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 8. 云台转回夹取位，别的舵机不动
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- LOAD_TRAY_2：夹取物料 → 云台转到托盘2号位 → 放到托盘2号位 ---------- */
static const PackStep_t g_steps_pkg1[] = {
    // 1. 滑轨从过渡位下降到物料夹取位（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_GRAB,              60},
    // 2. 夹爪从张开位回到零位夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨从夹取位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转到托盘位2，同时根据当前选择动态设置托盘舵机角度
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY2, TRAY_POS_DYNAMIC, 0,                        25},
    // 5. 滑轨从过渡位下降到物料托盘放置位（绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},
    // 6. 夹爪从零位回到张开位放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 7. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 8. 云台转回夹取位，别的舵机不动
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- LOAD_TRAY_1：夹取物料 → 云台转到托盘1号位 → 放到托盘1号位---------- */
static const PackStep_t g_steps_pkg3[] = {
    // 1. 滑轨从过渡位下降到物料夹取位（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_GRAB,              60},
    // 2. 夹爪从张开位回到零位夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨从夹取位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转到托盘位1，只动云台舵机，别的舵机不动
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY1, 0xFFFF,      0,                           25},
    // 5. 滑轨从过渡位下降到物料托盘放置位（绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},
    // 6. 夹爪从零位回到张开位放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 7. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 8. 云台转回夹取位，别的舵机不动
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- UNLOAD_TRAY_1：从托盘1取料 → 放到物料放置位7cm ---------- */
static const PackStep_t g_steps_pkg4[] = {
    // 1. 云台转到托盘1位
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY1, 0xFFFF,      0,                           25},
    // 2. 滑轨从过渡位下降到托盘夹取位10cm（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PICKUP,       40},
    // 3. 夹爪从张开位回到零位夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 4. 滑轨从托盘放置位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 5. 云台转回夹取位，只动云台
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 6. 滑轨从过渡位下降到物料放置位7cm（绝对位置5600）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_PLACE,             70},
    // 7. 夹爪从零位回到张开位放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 8. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           70},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- UNLOAD_TRAY_2：从托盘2取料 → 放到物料放置位置 ---------- */
static const PackStep_t g_steps_pkg6[] = {
    // 1. 根据方向键预选的角度，先将托盘转到对应格位（取料前对准），同时云台转到托盘位2
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY2, 0xFFFF,      0,                      20},
    // 2. 滑轨从过渡位下降到托盘夹取位10cm（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PICKUP,       40},
    // 3. 夹爪从张开位回到零位夹取物料
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 4. 滑轨从托盘放置位回到过渡位（绝对位置12000），夹爪带料上升
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 5. 云台转回夹取位，准备放到物料放置位
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 6. 滑轨从过渡位下降到物料放置位7cm（绝对位置5600）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_PLACE,             70},
    // 7. 夹爪从零位回到张开位放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 8. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           70},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- UNLOAD_TRAY_3：从托盘3取料 → 放到物料放置位置---------- */
static const PackStep_t g_steps_pkg7[] = {
    // 1. 云台转到托盘3位
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY3, 0xFFFF,      0,                           45},
    // 2. 滑轨下降到托盘夹取位10cm（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PICKUP,       40},
    // 3. 夹爪夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 4. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 5. 云台转回夹取位，只动云台
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 6. 滑轨下降到物料放置位7cm（绝对位置5600）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_PLACE,             70},
    // 7. 夹爪放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 8. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           70},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- LOAD_TRAY_2_FROM_PLACE：从物料放置位(零位)夹取 → 云台转到托盘2号位 → 放到托盘2号位 ----------
 * 与 LOAD_TRAY_2 完全相同，唯一区别：第1步滑轨下降到物料放置位(零位)夹取，而非物料夹取位 */
static const PackStep_t g_steps_pkg8[] = {
    // 1. 滑轨从过渡位下降到物料放置位（绝对位置0，零位）夹取
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_PLACE,             60},
    // 2. 夹爪从张开位回到零位夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨从夹取位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转到托盘位2，同时根据当前选择动态设置托盘舵机角度
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY2, TRAY_POS_DYNAMIC, 0,                        25},
    // 5. 滑轨从过渡位下降到物料托盘放置位（绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},
    // 6. 夹爪从零位回到张开位放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 7. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 8. 云台转回夹取位，别的舵机不动
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ============================================================
 *  包注册表：所有包的汇总
 * ============================================================ */
static const Package_t g_packages[] = {
    {g_steps_pkg3, sizeof(g_steps_pkg3)/sizeof(PackStep_t), 1},  // LOAD_TRAY_1
    {g_steps_pkg1, sizeof(g_steps_pkg1)/sizeof(PackStep_t), 1},  // LOAD_TRAY_2
    {g_steps_pkg0, sizeof(g_steps_pkg0)/sizeof(PackStep_t), 1},  // LOAD_TRAY_3
    {g_steps_pkg4, sizeof(g_steps_pkg4)/sizeof(PackStep_t), 1},  // UNLOAD_TRAY_1
    {g_steps_pkg6, sizeof(g_steps_pkg6)/sizeof(PackStep_t), 1},  // UNLOAD_TRAY_2
    {g_steps_pkg7, sizeof(g_steps_pkg7)/sizeof(PackStep_t), 1},  // UNLOAD_TRAY_3
    {g_steps_pkg8, sizeof(g_steps_pkg8)/sizeof(PackStep_t), 1},  // LOAD_TRAY_2_FROM_PLACE
};
#define PACKAGE_COUNT   (sizeof(g_packages) / sizeof(g_packages[0]))  // = 7

/* 托盘绝对位置别名：便于按角度语义做包内映射
 * TRAY_POS_1 (DEG_0)   = 一号托盘，绝对角度 120°
 * TRAY_POS_2 (DEG_90)  = 二号托盘，绝对角度 180°
 * TRAY_POS_3 (DEG_180) = 三号托盘，绝对角度 240°
 * TRAY_POS_4 (DEG_270) = 第四次循环的极限位置，绝对角度 270°
 */


/* 当前二号托盘目标角度，外部可手动覆写 */
uint16_t g_Tray2_CurrentAngle = TRAY_POS_DEG_0;

/* ============================================================
 *  内部函数：执行单步动作
 * ============================================================ */
static void Package_ExecuteStep(const PackStep_t *step)
{
    uint16_t claw = step->clawPos;
    uint16_t ptz  = step->ptzPos;
    uint16_t tray = step->trayPos;

    /* 包2 / 包5 需要按触发次数映射到 90° / 180° / 270° 的托盘格位 */
    if(tray != 0xFFFF) {
        if(g_pkg.currentPkgIdx == 2) {
            switch(g_pkg.completedTriggerCount) {
            case 0: tray = TRAY_POS_DEG_90;  break;
            case 1: tray = TRAY_POS_DEG_180; break;
            default: tray = TRAY_POS_DEG_270; break;
            }
        } else if(g_pkg.currentPkgIdx == 5) {
            switch(g_pkg.completedTriggerCount) {
            case 0: tray = TRAY_POS_DEG_270; break;
            case 1: tray = TRAY_POS_DEG_180; break;
            default: tray = TRAY_POS_DEG_90;  break;
            }
        }
    }

    /* 设置舵机（仅当值不为0xFFFF时执行） */
    if(claw != 0xFFFF) Servo_Claw_SetPulse(claw);
    if(ptz  != 0xFFFF) Servo_PTZ_SetPulse(ptz);
    if(tray == TRAY_POS_DYNAMIC) {
        tray = g_Tray2_CurrentAngle;
    }

    if(tray != 0xFFFF) Servo_Tray_SetPulse(tray);

    /* 设置滑轨 — 只要步骤类型是滑轨类就下发指令（含目标位置=0零位） */
    if(step->type == PACK_STEP_SLIDE || step->type == PACK_STEP_ALL)
    {
        // 绝对位置模式：slidePulses 是绝对位置脉冲数（正数）
        // raF=1 绝对位置模式
        uint32_t pulses = (uint32_t)step->slidePulses;

        /* 串口保护间隔（前）：先让总线静默一小段，确保此前可能刚发出的
         * 轮边电机命令已被驱动器按帧接收完毕，避免与下面的滑轨命令粘连。 */
        delay_ms(SLIDE_CMD_GUARD_MS);

        /* snF=0：不使用多机同步缓存，指令下发后立即执行。
         * 注意：若用 snF=1，该指令会进入驱动器同步等待区，必须等到一条
         * 广播同步命令(Emm_V5_Synchronous_motion_All)才会运动。本工程的
         * 广播同步只在轮边电机批量发送时触发，导致滑轨在“松开摇杆只做包动作”
         * 时缓存指令得不到触发 → 表现为偶发性“跳过指令、原地不动”。 */
        Emm_V5_Pos_Control(SLIDE_ADDR, 1, SLIDE_POS_VEL, SLIDE_POS_ACC,
                           pulses, 1, 0);  // raF=1 绝对位置模式 dir=1向零位上方（正方向），snF=0 立即执行

        /* 串口保护间隔（后）：发完滑轨命令再静默一小段，让驱动器靠空闲(IDLE)
         * 确认这条滑轨命令结束，再允许后续轮边命令上总线，杜绝撞帧丢命令。 */
        delay_ms(SLIDE_CMD_GUARD_MS);

        g_pkg.slideTriggered = 1;  // 标记有滑轨运动等待完成

    }
    else
    {
        g_pkg.slideTriggered = 0;
    }
}

/* ============================================================
 *  暴露接口实现
 * ============================================================ */

void Package_Init(void)
{
    g_pkg.state                 = PACK_STATE_IDLE;
    g_pkg.busy                  = 0;
    g_pkg.currentPkgIdx         = 0; // 初始化为包0
    g_pkg.currentStepIdx        = 0;
    g_pkg.completedTriggerCount = 0;
    g_pkg.waitTimer             = 0;
    g_pkg.slideTriggered        = 0;
}

void Package_Start(PackID_t id)
{
    if(g_pkg.busy) return;  // 正在执行，忽略

    if(id >= PACKAGE_COUNT) return;

    g_pkg.currentPkgIdx         = (uint8_t)id;
    g_pkg.currentStepIdx        = 0;
    g_pkg.completedTriggerCount = 0;
    g_pkg.waitTimer             = 0;
    g_pkg.slideTriggered        = 0;
    g_pkg.busy                  = 1;
    g_pkg.state                 = PACK_STATE_IDLE;
}

void Package_Tick(void)
{
    if(!g_pkg.busy) return;

    const Package_t *pkg = &g_packages[g_pkg.currentPkgIdx];
    const PackStep_t *step = &pkg->steps[g_pkg.currentStepIdx];

    switch(g_pkg.state)
    {
    case PACK_STATE_IDLE:
        /* 检查当前步骤是否为END */
        if(step->type == PACK_STEP_END)
        {
            /* 当前包执行完一次，记录完成次数；
             * 这里不自动继续下一轮，下一次按键再触发。 */
            g_pkg.completedTriggerCount++;
            g_pkg.busy = 0;
            g_pkg.state = PACK_STATE_IDLE;
            return;
        }

        /* 执行当前步骤 */
        Package_ExecuteStep(step);

        /* 如果类型为 DELAY 或需要等待滑轨，进入等待状态，否则立即切到下一步 */
        if(step->type == PACK_STEP_DELAY)
        {
            /* 纯延时步骤 */
            if(step->delayTicks > 0)
            {
                g_pkg.waitTimer = step->delayTicks;
                g_pkg.state = PACK_STATE_DELAY;
            }
            else
            {
                g_pkg.currentStepIdx++;
            }
        }
        else if(g_pkg.slideTriggered)
        {
            /* 有滑轨运动 → 等待滑轨到位（同时等待延时） */
            if(step->delayTicks > 0)
            {
                g_pkg.waitTimer = step->delayTicks;
            }
            else
            {
                /* 有滑轨但无延时，给一个默认超时 */
                g_pkg.waitTimer = 125;  // 约2.5s
            }
            g_pkg.state = PACK_STATE_WAIT_SLIDE;
        }
        else
        {
            /* 纯舵机步骤 → 只需等待延时 */
            if(step->delayTicks > 0)
            {
                g_pkg.waitTimer = step->delayTicks;
                g_pkg.state = PACK_STATE_DELAY;
            }
            else
            {
                /* 无延时，直接下一步 */
                g_pkg.currentStepIdx++;
            }
        }
        break;

    case PACK_STATE_WAIT_SLIDE:
        /* 等待超时结束（延时已经包含了滑轨运动时间） */
        if(g_pkg.waitTimer > 0)
        {
            g_pkg.waitTimer--;
        }
        else
        {
            g_pkg.currentStepIdx++;
            g_pkg.state = PACK_STATE_IDLE;
        }
        break;

    case PACK_STATE_DELAY:
        /* 等待延时结束 */
        if(g_pkg.waitTimer > 0)
        {
            g_pkg.waitTimer--;
        }
        else
        {
            g_pkg.currentStepIdx++;
            g_pkg.state = PACK_STATE_IDLE;
        }
        break;
    }
}

uint8_t Package_IsBusy(void)
{
    return g_pkg.busy;
}

void Set_Tray2_Target_Angle(uint16_t pwm_angle)
{
    g_Tray2_CurrentAngle = pwm_angle;
    Servo_Tray_SetPulse(g_Tray2_CurrentAngle);
}

void Package_Stop(void)
{
    g_pkg.busy                  = 0;
    g_pkg.state                 = PACK_STATE_IDLE;
    g_pkg.waitTimer             = 0;
    g_pkg.currentStepIdx        = 0;
    g_pkg.completedTriggerCount = 0;
}

uint8_t Package_GetCurrentIndex(void)
{
    return g_pkg.currentPkgIdx;
}
