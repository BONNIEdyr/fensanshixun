#ifndef __SERVO_PWM_H
#define __SERVO_PWM_H

#include "stm32f10x.h"
#include "config.h"

/**
 * @brief  初始化TIM8高级定时器，在PC6(CH1)/PC7(CH2)/PC8(CH3)上输出50Hz舵机PWM
 *         注意：TIM8是高级定时器，需额外调用TIM_CtrlPWMOutputs使能主输出
 *         PC6=夹爪(180°), PC7=云台(270°), PC8=托盘(270°)
 */
void Servo_PWM_Init(void);

/**
 * @brief  设置夹爪（PC6 = TIM8_CH1，180°舵机）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（CLAW_PWM_MIN ~ CLAW_PWM_MAX）
 */
void Servo_Claw_SetPulse(uint16_t pulse);

/**
 * @brief  设置云台（PC7 = TIM8_CH2，270°舵机）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（PTZ_PWM_MIN ~ PTZ_PWM_MAX）
 */
void Servo_PTZ_SetPulse(uint16_t pulse);

/**
 * @brief  设置托盘（PC8 = TIM8_CH3，270°舵机）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（TRAY_PWM_MIN ~ TRAY_PWM_MAX）
 */
void Servo_Tray_SetPulse(uint16_t pulse);

/**
 * @brief  三路TIM8舵机摆动控制（单次步进）
 * @param  pwmClaw   夹爪当前PWM值指针（会被更新）
 * @param  dirClaw   夹爪方向指针（会被更新）
 * @param  pwmPTZ    云台当前PWM值指针（会被更新）
 * @param  dirPTZ    云台方向指针（会被更新）
 * @param  pwmTray   托盘当前PWM值指针（会被更新）
 * @param  dirTray   托盘方向指针（会被更新）
 */
void Servo_Swing_Step(uint16_t *pwmClaw, uint8_t *dirClaw,
                      uint16_t *pwmPTZ, uint8_t *dirPTZ,
                      uint16_t *pwmTray, uint8_t *dirTray);

#endif


