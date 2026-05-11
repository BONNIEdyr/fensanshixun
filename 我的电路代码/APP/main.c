#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "server.h"
#include "ps2.h"

// 基础参数配置
#define JOYSTICK_CENTER        128
#define JOYSTICK_DEAD_ZONE      12
#define MOTOR_MAX_RPM         1000
#define MOTOR_ACC               10

// 电机正向转动定义（根据实际安装方向可能需要对调）
#define MOTOR_LEFT_FORWARD_DIR   0
#define MOTOR_RIGHT_FORWARD_DIR  1

/**
 * @brief  摇杆值转速度
 */
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

/**
 * @brief  限速保护
 */
static int16_t LimitMotorSpeed(int16_t speed)
{
	if(speed > MOTOR_MAX_RPM) return MOTOR_MAX_RPM;
	if(speed < -MOTOR_MAX_RPM) return -MOTOR_MAX_RPM;
	return speed;
}

/**
 * @brief  设置带方向的速度（带同步标志）
 */
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
	// 最后一个参数设为1，表示进入同步等待缓存
	Emm_V5_Vel_Control(addr, dir, vel, MOTOR_ACC, 1);
}

/**
 * @brief  全机立即停止
 */
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

/**
 * @brief  系统全局停止（含舵机）
 */
static void AllStop(const uint8_t motorAddr[4])
{
	Motor_AllStop(motorAddr);
	Servo_All_Stop();
}

/**
 * @brief  MAIN主程序
 */
int main(void)
{
	uint8_t i = 0;
	uint8_t servoRunning = 0;
	uint8_t ps2ControlEnabled = 0;
	
	int16_t forwardSpeed = 0;
	int16_t rotateSpeed = 0;
	// 平移
	int16_t strafeSpeed = 0; 
	
	int16_t v1, v2, v3, v4; 
	const uint8_t motorAddr[4] = {1, 2, 3, 4};
	PS2_JoystickTypeDef joystick;

	// 硬件初始化
	board_init();
	TIM8_PWM_Init();
	PS2_Init();

	// 初始状态强制停止
	for(i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 0);
		delay_ms(5);
	}
	Servo_All_Stop();

	// 等待驱动器初始化
	delay_ms(2000);

	while(1)
	{
		PS2_ScanKey(&joystick);

		// 1. 安全急停逻辑：同时按 R1 + R2
		if((joystick.btn2 & (PS2_BTN_R1 | PS2_BTN_R2)) == (PS2_BTN_R1 | PS2_BTN_R2))
		{
			AllStop(motorAddr);
			ps2ControlEnabled = 0;
			servoRunning = 0;
			delay_ms(20);
			continue;
		}

		// 2. 解锁逻辑：按下 START 键
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

		// 3. 读取手柄摇杆数据
		forwardSpeed = PS2_AxisToSpeed(joystick.LJoy_UD, 1); // 左摇杆上下：前进
		// 平移
		strafeSpeed  = PS2_AxisToSpeed(joystick.LJoy_LR, 0); // 左摇杆左右：横移
		rotateSpeed  = PS2_AxisToSpeed(joystick.RJoy_LR, 0); // 右摇杆左右：自转

		// 4. 麦克纳姆轮运动学解算
		// 平移
		v1 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed + rotateSpeed)); // 左前
		// 平移
		v2 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed - rotateSpeed)); // 右前
		// 平移
		v3 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed + rotateSpeed)); // 左后
		// 平移
		v4 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed - rotateSpeed)); // 右后

		// 5. 下发速度指令到电机缓存
		Motor_SetSignedSpeed(motorAddr[0], MOTOR_LEFT_FORWARD_DIR, v1);
		delay_ms(5);
		Motor_SetSignedSpeed(motorAddr[1], MOTOR_RIGHT_FORWARD_DIR, v2);
		delay_ms(5);
		Motor_SetSignedSpeed(motorAddr[2], MOTOR_LEFT_FORWARD_DIR, v3);
		delay_ms(5);
		Motor_SetSignedSpeed(motorAddr[3], MOTOR_RIGHT_FORWARD_DIR, v4);
		delay_ms(5);
		
		// 6. 发送同步命令，四个电机同时执行
		Emm_V5_Synchronous_motion(0);

		// 7. 舵机控制
		if((servoRunning == 0) &&
		   ((joystick.btn2 & (PS2_BTN_L1 | PS2_BTN_L2)) == (PS2_BTN_L1 | PS2_BTN_L2)))
		{
			Servo_All_Forward();
			servoRunning = 1;
		}

		delay_ms(20);
	}
}