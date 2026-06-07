#ifndef __PACKAGE_ACTION_H
#define __PACKAGE_ACTION_H

#include "stm32f10x.h"

#define TRAY_POS_DEG_0    250
#define TRAY_POS_DEG_90   583
#define TRAY_POS_DEG_180  917
#define TRAY_POS_DEG_270  1250
/* ============================================================
 *  包动作步骤类型
 * ============================================================ */
typedef enum {
    PACK_STEP_SERVO,          // 设置三个舵机位置
    PACK_STEP_SLIDE,          // 滑轨相对运动（正=CCW，负=CW）
    PACK_STEP_ALL,            // 同时设置舵机 + 滑轨
    PACK_STEP_DELAY,          // 仅延时
    PACK_STEP_END,            // 包结束标志（必须）
} PackStepType_t;

/* ============================================================
 *  单步动作结构体
 *  每个步骤会：设置指定执行器 → 等待 delayTicks 个主循环周期
 *  1 tick ≈ MAIN_LOOP_DELAY_MS (20ms)
 * ============================================================ */
typedef struct {
    PackStepType_t type;            // 步骤类型
    uint16_t       clawPos;         // 夹爪目标PWM (0xFFFF=不变)
    uint16_t       ptzPos;          // 云台目标PWM (0xFFFF=不变)
    uint16_t       trayPos;         // 托盘目标PWM (0xFFFF=不变)
    int32_t        slidePulses;     // 滑轨相对脉冲 (正=CCW, 负=CW, 0=不动)
    uint16_t       delayTicks;      // 本步后等待的tick数 (1tick≈20ms)
} PackStep_t;

/* ============================================================
 *  包定义结构体
 * ============================================================ */
typedef struct {
    const PackStep_t *steps;                  // 步骤数组
    uint8_t           stepCount;              // 步骤数
    uint8_t           requiredTriggerCount;   // 该包在切换到下一个包前，需要按键触发的次数
} Package_t;

/* ============================================================
 *  包序列初始化
 * ============================================================ */

/**
 * @brief  初始化包执行器（上电时调用一次）
 */
void Package_Init(void);

/**
 * @brief  包ID类型：用于离散调用单次执行包
 */
typedef enum {
    LOAD_TRAY_1 = 0,
    LOAD_TRAY_2,
    LOAD_TRAY_3,
    UNLOAD_TRAY_1,
    UNLOAD_TRAY_2,
    UNLOAD_TRAY_3,
    LOAD_TRAY_2_FROM_PLACE,   // 与 LOAD_TRAY_2 相同，但夹取时滑轨下降到物料放置位(零位)
} PackID_t;

/**
 * @brief  启动指定单次执行包
 */
void Package_Start(PackID_t id);

/**
 * @brief  覆写二号托盘目标角度并立即驱动托盘舵机
 */
void Set_Tray2_Target_Angle(uint16_t pwm_angle);

/**
 * @brief  主循环中周期性调用，驱动包按步骤执行
 * @note   每轮主循环调用一次，不阻塞
 */
void Package_Tick(void);

/**
 * @brief  查询包是否正在执行
 * @return 1=正在执行，0=空闲
 */
uint8_t Package_IsBusy(void);

/**
 * @brief  强制中止当前包执行
 */
void Package_Stop(void);

/**
 * @brief  获取当前包序号 (0~7)
 */
uint8_t Package_GetCurrentIndex(void);

#endif /* __PACKAGE_ACTION_H */
