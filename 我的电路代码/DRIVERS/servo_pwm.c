#include "servo_pwm.h"

/**
 * @brief  初始化TIM2，在PA0(CH1)和PA1(CH2)上输出50Hz舵机PWM
 *         定时时间=(arr+1)(psc+1)/Tclk=10000*144/72MHz=20ms=50Hz
 */
void Servo_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	/* 使能时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* 配置PA0和PA1为复用推挽输出 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* TIM2时基配置：周期20ms (50Hz) */
	TIM_TimeBaseStruct.TIM_Period = 9999;            // arr
	TIM_TimeBaseStruct.TIM_Prescaler = 143;          // psc
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

	/* PWM通道配置：模式1，高电平有效，初始占空比750（约90度） */
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = SERVO_PWM_MID_PULSE;

	/* CH1 - PA0 */
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* CH2 - PA1 */
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}
 

/**
 * @brief  设置舵机1（PA0）PWM脉冲宽度
 */
void Servo1_SetPulse(uint16_t pulse)
{
	TIM_SetCompare1(TIM2, pulse);
}

/**
 * @brief  设置舵机2（PA1）PWM脉冲宽度
 */
void Servo2_SetPulse(uint16_t pulse)
{
	TIM_SetCompare2(TIM2, pulse);
}

/**
 * @brief  TIM2舵机摆动控制（单次步进）
 *         根据当前PWM值和方向，步进并自动反向限位
 */
void Servo_Swing_Step(uint16_t *pwm1, uint8_t *dir1,
                      uint16_t *pwm2, uint8_t *dir2)
{
	/* 舵机1 (PA0, TIM2_CH1) 往复摆动 */
	if(*dir1) *pwm1 = *pwm1 + SERVO_PWM_STEP;
	else      *pwm1 = *pwm1 - SERVO_PWM_STEP;

	if(*pwm1 > SERVO_PWM_MAX_PULSE) *dir1 = 0;  // 超过上限反转
	if(*pwm1 < SERVO_PWM_MIN_PULSE) *dir1 = 1;  // 低于下限反转

	Servo1_SetPulse(*pwm1);

	/* 舵机2 (PA1, TIM2_CH2) 反向摆动 */
	if(*dir2) *pwm2 = *pwm2 + SERVO_PWM_STEP;
	else      *pwm2 = *pwm2 - SERVO_PWM_STEP;

	if(*pwm2 > SERVO_PWM_MAX_PULSE) *dir2 = 0;
	if(*pwm2 < SERVO_PWM_MIN_PULSE) *dir2 = 1;

	Servo2_SetPulse(*pwm2);
}