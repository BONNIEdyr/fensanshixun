#ifndef __SERVO_PWM_H
#define __SERVO_PWM_H

#include "stm32f10x.h"
#include "config.h"

/**
 * @brief  初始化TIM8高级定时器，在PC6(CH1)/PC7(CH2)/PC8(CH3)上输出50Hz舵机PWM
 *         注意：TIM8是高级定时器，需额外调用TIM_CtrlPWMOutputs使能主输出
 */
void Servo_PWM_Init(void);

/**
 * @brief  设置舵机1（PC6 = TIM8_CH1）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（SERVO_PWM_MIN ~ SERVO_PWM_MAX）
 */
void Servo1_SetPulse(uint16_t pulse);

/**
 * @brief  设置舵机2（PC7 = TIM8_CH2）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（SERVO_PWM_MIN ~ SERVO_PWM_MAX）
 */
void Servo2_SetPulse(uint16_t pulse);

/**
 * @brief  设置舵机3（PC8 = TIM8_CH3）PWM脉冲宽度
 * @param  pulse 脉冲宽度值（SERVO_PWM_MIN ~ SERVO_PWM_MAX）
 */
void Servo3_SetPulse(uint16_t pulse);

/**
 * @brief  三路TIM8舵机摆动控制（单次步进）
 * @param  pwm1    舵机1当前PWM值指针（会被更新）
 * @param  dir1    舵机1方向指针（会被更新）
 * @param  pwm2    舵机2当前PWM值指针（会被更新）
 * @param  dir2    舵机2方向指针（会被更新）
 * @param  pwm3    舵机3当前PWM值指针（会被更新）
 * @param  dir3    舵机3方向指针（会被更新）
 */
void Servo_Swing_Step(uint16_t *pwm1, uint8_t *dir1,
                      uint16_t *pwm2, uint8_t *dir2,
                      uint16_t *pwm3, uint8_t *dir3);

#endif


