#ifndef __SERVO_PWM_H
#define __SERVO_PWM_H

#include "stm32f10x.h"

/* 舵机角度与PWM映射 */
#define SERVO_PWM_MIN_PULSE    250    // 对应约0度
#define SERVO_PWM_MID_PULSE    750    // 对应约90度
#define SERVO_PWM_MAX_PULSE   1250    // 对应约180度
#define SERVO_PWM_STEP          50    // 每次摆动步进值

/**
 * @brief  初始化TIM2，在PA0(CH1)和PA1(CH2)上输出50Hz舵机PWM
 */
void Servo_PWM_Init(void);

/**
 * @brief  设置舵机1（PA0）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（250~1250）
 */
void Servo1_SetPulse(uint16_t pulse);

/**
 * @brief  设置舵机2（PA1）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（250~1250）
 */
void Servo2_SetPulse(uint16_t pulse);

/**
 * @brief  TIM2舵机摆动控制（单次步进）
 * @param  pwm1    舵机1当前PWM值指针（会被更新）
 * @param  dir1    舵机1方向指针（会被更新）
 * @param  pwm2    舵机2当前PWM值指针（会被更新）
 * @param  dir2    舵机2方向指针（会被更新）
 */
void Servo_Swing_Step(uint16_t *pwm1, uint8_t *dir1,
                      uint16_t *pwm2, uint8_t *dir2);

#endif


