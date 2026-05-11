#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "server.h"
#include "ps2.h"

#define JOYSTICK_CENTER        128
#define JOYSTICK_DEAD_ZONE      12
#define MOTOR_MAX_RPM         1000
#define MOTOR_ACC               10

#define MOTOR_LEFT_FORWARD_DIR   0
#define MOTOR_RIGHT_FORWARD_DIR  1

static int16_t PS2_AxisToSpeed(uint8_t value, uint8_t reverse)
{
	int16_t axis;

	if(reverse)
	{
		axis = (int16_t)JOYSTICK_CENTER - (int16_t)value;
	}
	else
	{
		axis = (int16_t)value - (int16_t)JOYSTICK_CENTER;
	}

	if((axis > -JOYSTICK_DEAD_ZONE) && (axis < JOYSTICK_DEAD_ZONE))
	{
		return 0;
	}

	return (int16_t)((axis * MOTOR_MAX_RPM) / 127);
}

static int16_t LimitMotorSpeed(int16_t speed)
{
	if(speed > MOTOR_MAX_RPM)
	{
		return MOTOR_MAX_RPM;
	}

	if(speed < -MOTOR_MAX_RPM)
	{
		return -MOTOR_MAX_RPM;
	}

	return speed;
}

static void Motor_SetSignedSpeed(uint8_t addr, uint8_t forwardDir, int16_t speed)
{
	uint8_t dir = forwardDir;
	uint16_t vel;

	if(speed < 0)
	{
		dir = (uint8_t)!forwardDir;
		speed = (int16_t)-speed;
	}

	vel = (uint16_t)speed;
	Emm_V5_Vel_Control(addr, dir, vel, MOTOR_ACC, 1);
}

static void Motor_AllStop(const uint8_t motorAddr[4])
{
	uint8_t i;

	for(i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 1);
		delay_ms(5);
	}

	Emm_V5_Synchronous_motion(0);
}

static void AllStop(const uint8_t motorAddr[4])
{
	Motor_AllStop(motorAddr);
	Servo_All_Stop();
}

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

/**
	*	@brief		MAIN����
	*	@param		��
	*	@retval		��
	*/
int main(void)
{
	uint8_t i = 0;
	uint8_t servoRunning = 0;
	uint8_t ps2ControlEnabled = 0;
	int16_t forwardSpeed = 0;
	int16_t rotateSpeed = 0;
	int16_t leftSpeed = 0;
	int16_t rightSpeed = 0;
	const uint8_t motorAddr[4] = {1, 2, 3, 4};
	PS2_JoystickTypeDef joystick;

/**********************************************************
***	��ʼ����������
**********************************************************/
	board_init();
	TIM8_PWM_Init();
	PS2_Init();

	for(i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 0);
		delay_ms(5);
	}
	Servo_All_Stop();

/**********************************************************
***	�ϵ���ʱ2��ȴ�Emm_V5.0�ջ���ʼ�����?
**********************************************************/	
	delay_ms(2000);

/**********************************************************
***	�ٶ�ģʽ���ĵ��ͬ������������CW���ٶ�1000RPM�����ٶ�10
**********************************************************/
	for(i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 0);
		delay_ms(5);
	}
	Servo_All_Stop();

/**********************************************************
***	WHILEѭ��
**********************************************************/	
	while(1)
	{
		PS2_ScanKey(&joystick);

		if((joystick.btn2 & (PS2_BTN_R1 | PS2_BTN_R2)) == (PS2_BTN_R1 | PS2_BTN_R2))
		{
			AllStop(motorAddr);
			ps2ControlEnabled = 0;
			servoRunning = 0;
			delay_ms(20);
			continue;
		}

		if(ps2ControlEnabled == 0)
		{
			Motor_AllStop(motorAddr);

			if(joystick.btn1 & PS2_BTN_START)
			{
				ps2ControlEnabled = 1;
			}

			delay_ms(20);
			continue;
		}

		forwardSpeed = PS2_AxisToSpeed(joystick.LJoy_UD, 1);
		rotateSpeed = PS2_AxisToSpeed(joystick.RJoy_LR, 0);
		leftSpeed = LimitMotorSpeed((int16_t)(forwardSpeed + rotateSpeed));
		rightSpeed = LimitMotorSpeed((int16_t)(forwardSpeed - rotateSpeed));

		Motor_SetSignedSpeed(motorAddr[0], MOTOR_LEFT_FORWARD_DIR, leftSpeed);
		delay_ms(5);
		Motor_SetSignedSpeed(motorAddr[1], MOTOR_RIGHT_FORWARD_DIR, rightSpeed);
		delay_ms(5);
		Motor_SetSignedSpeed(motorAddr[2], MOTOR_LEFT_FORWARD_DIR, leftSpeed);
		delay_ms(5);
		Motor_SetSignedSpeed(motorAddr[3], MOTOR_RIGHT_FORWARD_DIR, rightSpeed);
		delay_ms(5);
		Emm_V5_Synchronous_motion(0);

		if((servoRunning == 0) &&
		   ((joystick.btn2 & (PS2_BTN_L1 | PS2_BTN_L2)) == (PS2_BTN_L1 | PS2_BTN_L2)))
		{
			Servo_All_Forward();
			servoRunning = 1;
		}

		delay_ms(20);
	}
}
