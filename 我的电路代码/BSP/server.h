#ifndef __SERVER_H
#define __SERVER_H

#include "stm32f10x.h"

#define SERVO_STOP_PULSE       15
#define SERVO_FORWARD_PULSE     5
#define SERVO_REVERSE_PULSE    25

void TIM8_PWM_Init(void);
void Servo_All_SetPulse(uint16_t pulse);
void Servo_All_Forward(void);
void Servo_All_Stop(void);
void Servo_All_Reverse(void);

#endif
