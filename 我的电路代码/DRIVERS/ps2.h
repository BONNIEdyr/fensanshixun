#ifndef __PS2_H
#define __PS2_H

#include "stm32f10x.h"

#define PS2_BTN_L2      0x01
#define PS2_BTN_R2      0x02
#define PS2_BTN_L1      0x04
#define PS2_BTN_R1      0x08
#define PS2_BTN_START   0x08

typedef struct
{
	uint8_t mode;
	uint8_t btn1;
	uint8_t btn2;
	uint8_t RJoy_LR;
	uint8_t RJoy_UD;
	uint8_t LJoy_LR;
	uint8_t LJoy_UD;
} PS2_JoystickTypeDef;

void PS2_Init(void);
void PS2_ScanKey(PS2_JoystickTypeDef *joystick);

#endif
