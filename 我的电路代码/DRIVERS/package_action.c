#include "package_action.h"
#include "servo_pwm.h"
#include "Emm_V5.h"
#include "config.h"

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
    PackState_t  state;             // 状态机状态
    uint8_t      currentPkgIdx;     // 当前执行到第几个包 (0~7)
    uint8_t      currentStepIdx;    // 当前执行到包内的第几步
    uint8_t      currentRepeat;     // 当前重复计数
    uint16_t     waitTimer;         // 延时/等待计时器 (tick)
    uint8_t      busy;              // 忙标志
    uint8_t      slideTriggered;    // 本步是否已触发了滑轨运动
} g_pkg;

/* ============================================================
 *  包0 ~ 包7 的步骤定义
 *  注释格式：
 *    // N. 动作描述
 *  滑轨4个高度：过渡位15cm / 托盘放置位13cm / 物料夹取位10cm / 物料放置位7cm
 *  夹爪：CLAW_POS_GRIP=0°闭合夹取 / CLAW_POS_RELEASE=60°张开放料
 *  云台：PTZ_POS_GRIP=0°夹取位 / PTZ_POS_TRAY1/2/3=托盘位1/2/3
 *  托盘：TRAY_POS_1/2/3/4=托盘1/2/3/4号位
 *  滑轨绝对位置宏（脉冲数）：
 *    SLIDE_POS_TRANSIT       = 12000（过渡位15cm）
 *    SLIDE_POS_TRAY_PLACE    = 10400（托盘放置位13cm）
 *    SLIDE_POS_GRAB          =  8000（物料夹取位10cm）
 *    SLIDE_POS_PLACE         =  5600（物料放置位7cm）
 *   所有滑轨控制使用绝对位置模式 (raF=1)
 * ============================================================ */

/* ---------- 包0：夹取物料 → 云台转到托盘3号位 → 放到托盘3号位 ---------- */
static const PackStep_t g_steps_pkg0[] = { 
    // 1. 滑轨从过渡位下降到物料夹取位（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_GRAB,              60},
    // 2. 夹爪从张开位(60°)回到零位(0°)夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨从夹取位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转到托盘位3，只动云台舵机，别的舵机不动
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY3, 0xFFFF,      0,                           25},
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

/* ---------- 包1：夹取物料 → 云台转到托盘2号位 → 放到托盘2号位 ---------- */
static const PackStep_t g_steps_pkg1[] = {
    // 1. 滑轨从过渡位下降到物料夹取位（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_GRAB,              60},
    // 2. 夹爪从张开位回到零位夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨从夹取位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转到托盘位2，只动云台舵机，别的舵机不动
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY2, 0xFFFF,      0,                           25},
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

/* ---------- 包2：夹取物料 → 云台转到托盘2号位 → 放到托盘2号位（重复3次）---------- */
static const PackStep_t g_steps_pkg2[] = {
    // 1. 滑轨从过渡位下降到物料夹取位（绝对位置8000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_GRAB,              60},
    // 2. 夹爪从张开位回到零位夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨从夹取位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转到托盘位2，同时托盘舵机转动90度
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY2, TRAY_POS_2,  0,                           25},
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

/* ---------- 包3：夹取物料 → 云台转到托盘1号位 → 放到托盘1号位---------- */
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
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- 包4：从托盘1取料 → 放到物料放置位7cm 云台转到2号位---------- */
static const PackStep_t g_steps_pkg4[] = {
    // 2. 滑轨从过渡位下降到物料托盘放置位13cm（绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},
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
    // 9. 云台转到2号位
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY2, 0xFFFF,      0,                           25},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- 包5：从托盘2取料 → 放到物料放置位置 → 重复三次---------- */
static const PackStep_t g_steps_pkg5[] = {
    // 2. 滑轨从过渡位下降到物料托盘放置位13cm（绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},
    // 3. 夹爪从张开位回到零位夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 4. 滑轨从托盘放置位回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 5. 云台转到夹取位，同时托盘舵机反方向转动90度
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  TRAY_POS_1,  0,                           25},
    // 6. 滑轨从过渡位下降到物料放置位7cm（绝对位置5600）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_PLACE,             70},
    // 7. 夹爪从零位回到张开位放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 8. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           70},
    // 9. 云台转回2号位
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY2, 0xFFFF,      0,                           25},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- 包6：从托盘2取料 → 放到物料放置位置--------- */
static const PackStep_t g_steps_pkg6[] = {
    // 2. 滑轨下降到托盘放置位13cm（绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},
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
    // 8. 滑轨回过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           70},
    // 9. 云台转到托盘3号位
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_TRAY3, 0xFFFF,      0,                           25},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ---------- 包7：从托盘3取料 → 放到物料放置位置---------- */
static const PackStep_t g_steps_pkg7[] = {
    // 1. 滑轨下降到托盘放置位13cm（从托盘3取料，绝对位置10400）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRAY_PLACE,        40},
    // 2. 夹爪夹取
    {PACK_STEP_SERVO,   CLAW_POS_GRIP,   0xFFFF,        0xFFFF,      0,                           15},
    // 3. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           60},
    // 4. 云台转回夹取位，只动云台
    {PACK_STEP_SERVO,   0xFFFF,          PTZ_POS_GRIP,  0xFFFF,      0,                           25},
    // 5. 滑轨下降到物料放置位7cm（绝对位置5600）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_PLACE,             70},
    // 6. 夹爪放料
    {PACK_STEP_SERVO,   CLAW_POS_RELEASE, 0xFFFF,       0xFFFF,      0,                           15},
    // 7. 滑轨回到过渡位（绝对位置12000）
    {PACK_STEP_SLIDE,   0xFFFF,          0xFFFF,        0xFFFF,      SLIDE_POS_TRANSIT,           70},
    // 结束
    {PACK_STEP_END,     0, 0, 0, 0, 0},
};

/* ============================================================
 *  包注册表：所有包的汇总
 * ============================================================ */
static const Package_t g_packages[] = {
    {g_steps_pkg0, sizeof(g_steps_pkg0)/sizeof(PackStep_t), 1},  // 包0 - 夹取→放托盘3号位
    {g_steps_pkg1, sizeof(g_steps_pkg1)/sizeof(PackStep_t), 1},  // 包1 - 夹取→放托盘2号位
    {g_steps_pkg2, sizeof(g_steps_pkg2)/sizeof(PackStep_t), 3},  // 包2 - 夹取→放托盘2号位（按3次进入下一个包）
    {g_steps_pkg3, sizeof(g_steps_pkg3)/sizeof(PackStep_t), 1},  // 包3 - 夹取→放托盘1号位（按1次进入下一个包）
    {g_steps_pkg4, sizeof(g_steps_pkg4)/sizeof(PackStep_t), 1},  // 包4 - 从托盘1取料→放到物料放置位7cm
    {g_steps_pkg5, sizeof(g_steps_pkg5)/sizeof(PackStep_t), 3},  // 包5 - 从托盘2取料→放到物料放置位置（按3次进入下一个包）
    {g_steps_pkg6, sizeof(g_steps_pkg6)/sizeof(PackStep_t), 1},  // 包6 - 从托盘2取料→放到物料放置位
    {g_steps_pkg7, sizeof(g_steps_pkg7)/sizeof(PackStep_t), 1},  // 包7 - 从托盘3取料→放到物料放置位置
};
#define PACKAGE_COUNT   (sizeof(g_packages) / sizeof(g_packages[0]))  // = 8

/* ============================================================
 *  内部函数：执行单步动作
 * ============================================================ */
static void Package_ExecuteStep(const PackStep_t *step)
{
    uint16_t claw = step->clawPos;
    uint16_t ptz  = step->ptzPos;
    uint16_t tray = step->trayPos;

    /* 设置舵机（仅当值不为0xFFFF时执行） */
    if(claw != 0xFFFF) Servo_Claw_SetPulse(claw);
    if(ptz  != 0xFFFF) Servo_PTZ_SetPulse(ptz);
    if(tray != 0xFFFF) Servo_Tray_SetPulse(tray);

    /* 设置滑轨（仅当不为0时执行）- 绝对位置模式 raF=1 */
    if(step->slidePulses != 0)
    {
        // 绝对位置模式：slidePulses 是绝对位置脉冲数（正数）
        // raF=1 绝对位置模式
        uint32_t pulses = (uint32_t)step->slidePulses;

        Emm_V5_Pos_Control(SLIDE_ADDR, 0, SLIDE_POS_VEL, SLIDE_POS_ACC,
                           pulses, 0, 1);  // raF=1 绝对位置模式
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
    g_pkg.state    = PACK_STATE_IDLE;
    g_pkg.busy     = 0;
    g_pkg.currentPkgIdx  = PACKAGE_COUNT - 1;
    g_pkg.currentStepIdx = 0;
    g_pkg.currentRepeat  = 0;
    g_pkg.waitTimer      = 0;
    g_pkg.slideTriggered = 0;
}

void Package_StartNext(void)
{
    if(g_pkg.busy) return;  // 正在执行，忽略

    /* 第一次启动或当前包已完成指定次数后，切换到下一个包 */
    if(g_pkg.currentRepeat == 0 && g_pkg.currentStepIdx == 0 && g_pkg.waitTimer == 0)
    {
        g_pkg.currentPkgIdx = (g_pkg.currentPkgIdx + 1) % PACKAGE_COUNT;
    }
    else if(g_pkg.currentRepeat >= g_packages[g_pkg.currentPkgIdx].repeatCount)
    {
        g_pkg.currentPkgIdx = (g_pkg.currentPkgIdx + 1) % PACKAGE_COUNT;
        g_pkg.currentRepeat = 0;
    }

    g_pkg.currentStepIdx = 0;
    g_pkg.waitTimer      = 0;
    g_pkg.slideTriggered = 0;
    g_pkg.busy = 1;
    g_pkg.state = PACK_STATE_IDLE;
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
            g_pkg.currentRepeat++;
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

void Package_Stop(void)
{
    g_pkg.busy         = 0;
    g_pkg.state        = PACK_STATE_IDLE;
    g_pkg.waitTimer    = 0;
    g_pkg.currentStepIdx = 0;
    g_pkg.currentRepeat  = 0;
}

uint8_t Package_GetCurrentIndex(void)
{
    return g_pkg.currentPkgIdx;
}
