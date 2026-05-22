#include "servo_pwm.h"

/**
 * @brief  初始化TIM8高级定时器，在PC6(CH1)/PC7(CH2)/PC8(CH3)上输出50Hz舵机PWM
 *         PC6=夹爪(180°), PC7=云台(270°), PC8=托盘(270°)
 *         注意：TIM8是高级定时器，需调用TIM_CtrlPWMOutputs使能主输出
 *         定时时间=(ARR+1)(PSC+1)/Tclk=(9999+1)*(143+1)/72MHz=20ms=50Hz
 */
void Servo_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	/* 使能时钟：GPIOC + TIM8（均在APB2上） */
	RCC_APB2PeriphClockCmd(SERVO_TIM_GPIO_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(SERVO_TIM_RCC, ENABLE);

	/* 配置PC6(夹爪)、PC7(云台)、PC8(托盘)为复用推挽输出 */
	GPIO_InitStruct.GPIO_Pin = SERVO_CLAW_PIN | SERVO_PTZ_PIN | SERVO_TRAY_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SERVO_GPIO, &GPIO_InitStruct);

	/* TIM8时基配置：周期20ms (50Hz) */
	TIM_TimeBaseStruct.TIM_Period = SERVO_ARR;            // arr = 9999
	TIM_TimeBaseStruct.TIM_Prescaler = SERVO_PSC;         // psc = 143
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStruct);

	/* PWM通道配置：模式1，高电平有效，初始占空比为SERVO_PWM_MID（约90度） */
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = SERVO_PWM_MID;

	/* CH1 - PC6 */
	TIM_OC1Init(SERVO_TIM, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Disable);

	/* CH2 - PC7 */
	TIM_OC2Init(SERVO_TIM, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(SERVO_TIM, TIM_OCPreload_Disable);

	/* CH3 - PC8 */
	TIM_OC3Init(SERVO_TIM, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(SERVO_TIM, TIM_OCPreload_Disable);

	TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);

	/* 重要：TIM8是高级定时器，必须调用此函数使能PWM主输出 */
	TIM_CtrlPWMOutputs(SERVO_TIM, ENABLE);

	TIM_Cmd(SERVO_TIM, ENABLE);
}

/**
 * @brief  设置夹爪（PC6 = TIM8_CH1，180°舵机）PWM脉冲宽度
 */
void Servo_Claw_SetPulse(uint16_t pulse)
{
	TIM_SetCompare1(SERVO_TIM, pulse);
}

/**
 * @brief  设置云台（PC7 = TIM8_CH2，270°舵机）PWM脉冲宽度
 */
void Servo_PTZ_SetPulse(uint16_t pulse)
{
	TIM_SetCompare2(SERVO_TIM, pulse);
}

/**
 * @brief  设置托盘（PC8 = TIM8_CH3，270°舵机）PWM脉冲宽度
 */
void Servo_Tray_SetPulse(uint16_t pulse)
{
	TIM_SetCompare3(SERVO_TIM, pulse);
}

/**
 * @brief  三路TIM8舵机摆动控制（单次步进）
 *         夹爪(PC6)、云台(PC7)、托盘(PC8) 以不同的方向/相位往复摆动
 */
void Servo_Swing_Step(uint16_t *pwmClaw, uint8_t *dirClaw,
                      uint16_t *pwmPTZ, uint8_t *dirPTZ,
                      uint16_t *pwmTray, uint8_t *dirTray)
{
	/* 夹爪 (PC6, TIM8_CH1, 180°舵机) 摆动 */
	if(*dirClaw) *pwmClaw = *pwmClaw + SERVO_PWM_STEP;
	else         *pwmClaw = *pwmClaw - SERVO_PWM_STEP;

	if(*pwmClaw > SERVO_PWM_MAX) *dirClaw = 0;
	if(*pwmClaw < SERVO_PWM_MIN) *dirClaw = 1;

	Servo_Claw_SetPulse(*pwmClaw);

	/* 云台 (PC7, TIM8_CH2, 270°舵机) 与夹爪反向摆动 */
	if(*dirPTZ) *pwmPTZ = *pwmPTZ + SERVO_PWM_STEP;
	else        *pwmPTZ = *pwmPTZ - SERVO_PWM_STEP;

	if(*pwmPTZ > SERVO_PWM_MAX) *dirPTZ = 0;
	if(*pwmPTZ < SERVO_PWM_MIN) *dirPTZ = 1;

	Servo_PTZ_SetPulse(*pwmPTZ);

	/* 托盘 (PC8, TIM8_CH3, 270°舵机) 与夹爪同向摆动 */
	if(*dirTray) *pwmTray = *pwmTray + SERVO_PWM_STEP;
	else         *pwmTray = *pwmTray - SERVO_PWM_STEP;

	if(*pwmTray > SERVO_PWM_MAX) *dirTray = 0;
	if(*pwmTray < SERVO_PWM_MIN) *dirTray = 1;

	Servo_Tray_SetPulse(*pwmTray);
}
