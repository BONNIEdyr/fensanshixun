#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "server.h"
#include "ps2.h"
#include "config.h"

/* 将PS2摇杆的原始值转换为电机目标转速。reverse用于修正摇杆方向。 */
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

/* 限制电机速度，防止超过配置的最大转速。 */
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

/* 按带符号速度控制单个电机，正负号决定实际转向。 */
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

/* 停止全部电机，并发送同步执行命令。 */
static void Motor_AllStop(const uint8_t motorAddr[MOTOR_COUNT])
{
	uint8_t i;

	for(i = 0; i < MOTOR_COUNT; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 1);
		delay_ms(MOTOR_CMD_DELAY_MS);
	}

	Emm_V5_Synchronous_motion(0);
}

/* 紧急停止：同时停止电机和舵机。 */
static void AllStop(const uint8_t motorAddr[MOTOR_COUNT])
{
	Motor_AllStop(motorAddr);
	Servo_All_Stop();
}

/**
 * @brief  主程序入口
 * @param  无
 * @retval 无
 *
 * 控制逻辑：
 * 1. 初始化串口、电机、舵机PWM和PS2手柄。
 * 2. 按START键进入手柄控制模式。
 * 3. 左摇杆上下控制前进/后退，右摇杆左右控制转向。
 * 4. 同时按R1和R2执行急停并退出手柄控制模式。
 * 5. 同时按L1和L2启动舵机正转。
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
	const uint8_t motorAddr[MOTOR_COUNT] = {MOTOR_ADDR_1, MOTOR_ADDR_2, MOTOR_ADDR_3, MOTOR_ADDR_4};
	PS2_JoystickTypeDef joystick;

	/* 初始化板级资源、舵机PWM和PS2手柄接口。 */
	board_init();
	TIM8_PWM_Init();
	PS2_Init();

	/* 上电后先让所有电机和舵机保持停止状态。 */
	for(i = 0; i < MOTOR_COUNT; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 0);
		delay_ms(MOTOR_CMD_DELAY_MS);
	}
	Servo_All_Stop();

	/* 等待电机驱动器完成上电初始化。 */
	delay_ms(SYSTEM_START_DELAY_MS);

	/* 初始化完成后再次停止全部执行机构，确保进入主循环前状态安全。 */
	for(i = 0; i < MOTOR_COUNT; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 0);
		delay_ms(MOTOR_CMD_DELAY_MS);
	}
	Servo_All_Stop();

	/* 主循环：持续读取手柄输入并输出电机、舵机控制命令。 */
	while(1)
	{
		PS2_ScanKey(&joystick);

		/* R1+R2：急停，并退出手柄控制模式。 */
		if((joystick.btn2 & (PS2_BTN_R1 | PS2_BTN_R2)) == (PS2_BTN_R1 | PS2_BTN_R2))
		{
			AllStop(motorAddr);
			ps2ControlEnabled = 0;
			servoRunning = 0;
			delay_ms(MAIN_LOOP_DELAY_MS);
			continue;
		}

		/* 未进入控制模式时保持电机停止；按START后允许手柄控制。 */
		if(ps2ControlEnabled == 0)
		{
			Motor_AllStop(motorAddr);

			if(joystick.btn1 & PS2_BTN_START)
			{
				ps2ControlEnabled = 1;
			}

			delay_ms(MAIN_LOOP_DELAY_MS);
			continue;
		}

		/* 左摇杆上下映射为前进速度，右摇杆左右映射为转向速度。 */
		forwardSpeed = PS2_AxisToSpeed(joystick.LJoy_UD, 1);
		rotateSpeed = PS2_AxisToSpeed(joystick.RJoy_LR, 0);
		leftSpeed = LimitMotorSpeed((int16_t)(forwardSpeed + rotateSpeed));
		rightSpeed = LimitMotorSpeed((int16_t)(forwardSpeed - rotateSpeed));

		/* 四个电机分别下发速度，最后统一同步启动。 */
		Motor_SetSignedSpeed(motorAddr[0], MOTOR_LEFT_FORWARD_DIR, leftSpeed);
		delay_ms(MOTOR_CMD_DELAY_MS);
		Motor_SetSignedSpeed(motorAddr[1], MOTOR_RIGHT_FORWARD_DIR, rightSpeed);
		delay_ms(MOTOR_CMD_DELAY_MS);
		Motor_SetSignedSpeed(motorAddr[2], MOTOR_LEFT_FORWARD_DIR, leftSpeed);
		delay_ms(MOTOR_CMD_DELAY_MS);
		Motor_SetSignedSpeed(motorAddr[3], MOTOR_RIGHT_FORWARD_DIR, rightSpeed);
		delay_ms(MOTOR_CMD_DELAY_MS);
		Emm_V5_Synchronous_motion(0);

		/* L1+L2：只触发一次舵机正转，避免循环中重复发送同一命令。 */
		if((servoRunning == 0) &&
		   ((joystick.btn2 & (PS2_BTN_L1 | PS2_BTN_L2)) == (PS2_BTN_L1 | PS2_BTN_L2)))
		{
			Servo_All_Forward();
			servoRunning = 1;
		}

		delay_ms(MAIN_LOOP_DELAY_MS);
	}
}
