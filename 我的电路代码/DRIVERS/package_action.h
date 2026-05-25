#ifndef __PACKAGE_ACTION_H
#define __PACKAGE_ACTION_H

#include "stm32f10x.h"

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
    const PackStep_t *steps;        // 步骤数组
    uint8_t           stepCount;    // 步骤数
    uint8_t           repeatCount;  // 该包在切换到下一个包前，需要按键执行的次数
} Package_t;

/* ============================================================
 *  包序列初始化
 * ============================================================ */

/**
 * @brief  初始化包执行器（上电时调用一次）
 */
void Package_Init(void);

/**
 * @brief  启动包执行（L1+L2按下时调用）
 *         每次调用只执行当前包的一次；
 *         若该包配置了多次触发次数，则会保留当前包，直到触发次数满足后再切换到下一个包
 */
void Package_StartNext(void);

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
