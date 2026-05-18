#ifndef __SERVER_H
#define __SERVER_H

#include "stm32f10x.h"
#include "config.h"

void TIM8_PWM_Init(void);
void Servo_Gimbal_SetAngle(uint16_t angle);
void Servo_Tray_SetAngle(uint16_t angle);
void Servo_Claw_SetAngle(uint16_t angle);
void Servo_All_SetZero(void);
void Servo_All_Stop(void);
void Servo_All_Forward(void);

#endif
