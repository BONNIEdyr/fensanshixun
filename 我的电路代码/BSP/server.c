#include "server.h"

void TIM8_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(SERVO_TIM_RCC | SERVO_GPIO_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = SERVO_CH1_PIN | SERVO_CH2_PIN | SERVO_CH3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SERVO_GPIO, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = SERVO_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = SERVO_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = SERVO_STOP_PULSE;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	TIM_OC1Init(SERVO_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);

	TIM_OC2Init(SERVO_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);

	TIM_OC3Init(SERVO_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);
	TIM_Cmd(SERVO_TIM, ENABLE);
	TIM_CtrlPWMOutputs(SERVO_TIM, ENABLE);
}

void Servo_All_SetPulse(uint16_t pulse)
{
	TIM_SetCompare1(SERVO_TIM, pulse);
	TIM_SetCompare2(SERVO_TIM, pulse);
	TIM_SetCompare3(SERVO_TIM, pulse);
}

void Servo_All_Forward(void)
{
	Servo_All_SetPulse(SERVO_FORWARD_PULSE);
}

void Servo_All_Stop(void)
{
	Servo_All_SetPulse(SERVO_STOP_PULSE);
}

void Servo_All_Reverse(void)
{
	Servo_All_SetPulse(SERVO_REVERSE_PULSE);
}
