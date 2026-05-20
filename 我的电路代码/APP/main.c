#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "ps2.h"
#include "config.h"
#include "servo_pwm.h"

/**
 * @brief  摇杆值转速度
 * @param  value    摇杆原始ADC值
 * @param  reverse  是否反转方向
 * @param  maxSpeed 该轴的最大速度限制值
 */
static int16_t PS2_AxisToSpeed(uint8_t value, uint8_t reverse, int16_t maxSpeed)
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

	return (int16_t)((axis * maxSpeed) / 127);
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
 * @brief  系统全局停止
 */
static void AllStop(const uint8_t motorAddr[4])
{
	Motor_AllStop(motorAddr);
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
	int16_t strafeSpeed = 0; 
	
	int16_t v1, v2, v3, v4; 
	const uint8_t motorAddr[4] = {1, 2, 3, 4};
	PS2_JoystickTypeDef joystick;

	/* 摇杆归零急停计时器（非阻塞，避免 delay_ms 阻塞 PS2_ScanKey） */
	uint16_t zeroStopTimer = 0;
	uint8_t  zeroStopArmed = 0;   // 0=未启动, 1=已启动计时

	/* ----- 舵机摆动控制变量（TIM8/PC6-PC8）----- */
	uint16_t servoPWM1 = 750;    // 舵机1（PC6=TIM8_CH1）当前PWM，750≈90度中位
	uint16_t servoPWM2 = 750;    // 舵机2（PC7=TIM8_CH2）当前PWM
	uint16_t servoPWM3 = 750;    // 舵机3（PC8=TIM8_CH3）当前PWM
	uint8_t  servoDir1  = 1;     // 舵机1方向：1增大，0减小
	uint8_t  servoDir2  = 0;     // 舵机2与1反向
	uint8_t  servoDir3  = 1;     // 舵机3与1同向

	/* ----- 滑轨电机5控制变量 ----- */
	uint8_t  slideHomed = 0;     // 滑轨回零完成标志：0未完成，1已完成
	uint8_t  slideTriggered = 0; // 滑轨10cm运动已触发标志，防止重复触发
	uint8_t  prevBtn2 = 0;       // 上一次的btn2值，用于按键边沿检测

	// 硬件初始化
	board_init();
	Servo_PWM_Init();            // 初始化TIM8舵机PWM（PC6_CH1/PC7_CH2/PC8_CH3）
	PS2_Init();

	// 初始状态强制停止
	for(i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 0);
		delay_ms(5);
	}

	// 等待驱动器初始化
	delay_ms(2000);

	/* ===== 滑轨电机5上电自动回零 ===== */
	Emm_V5_Origin_Modify_Params(
		SLIDE_ADDR,        // 地址5
		1,                 // svF=1 存储（每次上电都回零）
		2,                 // o_mode=2 多圈无限位碰撞回零
		0,                 // o_dir=0 CW方向走到底部
		SLIDE_HOMING_VEL,  // 回零速度200RPM
		SLIDE_HOMING_TIMEOUT_MS, // 超时5秒
		SLIDE_SL_VEL,      // 碰撞检测转速50RPM
		SLIDE_SL_MA,       // 碰撞检测电流500mA
		SLIDE_SL_MS,       // 碰撞检测维持时间500ms
		0);                // potF=0 不上电自动触发（我们手动触发）
	delay_ms(MOTOR_CMD_DELAY_MS);

	// 触发回零
	Emm_V5_Origin_Trigger_Return(SLIDE_ADDR, 2, 0);
	// 等待回零完成（阻塞等待，此处延时需大于回零实际耗时）
	delay_ms(SLIDE_HOMING_TIMEOUT_MS + 1000);

	// 回零完成后，将当前位置标记为绝对零点
	Emm_V5_Reset_CurPos_To_Zero(SLIDE_ADDR);
	delay_ms(MOTOR_CMD_DELAY_MS);

	// 确保滑轨电机静止
	Emm_V5_Stop_Now(SLIDE_ADDR, 0);
	delay_ms(MOTOR_CMD_DELAY_MS);
	slideHomed = 1;  // 标记回零完成

	while(1)
	{
		PS2_ScanKey(&joystick);

		// 0. 模式门控：只有绿灯模拟模式(0x73)才执行控制逻辑
		//    红灯模式或断连时停止电机并上锁
		if(joystick.mode != 0x73)
		{
			AllStop(motorAddr);
			ps2ControlEnabled = 0;
			zeroStopArmed = 0;   // 清除急停计时
			delay_ms(20);
			continue;
		}

		// 1. 安全急停逻辑：同时按 R1 + R2
		if((joystick.btn2 & (PS2_BTN_R1 | PS2_BTN_R2)) == (PS2_BTN_R1 | PS2_BTN_R2))
		{
			AllStop(motorAddr);
			ps2ControlEnabled = 0;
			servoRunning = 0;
			zeroStopArmed = 0;
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
			zeroStopArmed = 0;
			delay_ms(20);
			continue;
		}

		// 3. 读取手柄摇杆数据
		forwardSpeed = PS2_AxisToSpeed(joystick.LJoy_UD, 1, MOTOR_MAX_RPM);   // 左摇杆上下：前进
		strafeSpeed  = PS2_AxisToSpeed(joystick.LJoy_LR, 0, MOTOR_MAX_RPM);    // 左摇杆左右：横移
		rotateSpeed  = PS2_AxisToSpeed(joystick.RJoy_LR, 0, RJOYSTICK_MAX_SPEED); // 右摇杆左右：自转

		// 4. 摇杆归零急停逻辑（非阻塞）
		//    松手回中 → 立即发送速度0让电机按MOTOR_ACC自然减速
		//    同时启动计时器，计时到后彻底锁死
		//    如果在计时期间摇杆再次有值 → 取消计时，恢复正常控制
		if(forwardSpeed == 0 && strafeSpeed == 0 && rotateSpeed == 0)
		{
			// 首次进入 → 立即发送速度0，让电机按MOTOR_ACC减速
			if(zeroStopArmed == 0)
			{
				zeroStopArmed = 1;
				zeroStopTimer = 0;

				// 立即给所有电机发速度0（带MOTOR_ACC，自然减速）
				Emm_V5_Vel_Control(motorAddr[0], 0, 0, MOTOR_ACC, 1);
				delay_ms(5);
				Emm_V5_Vel_Control(motorAddr[1], 0, 0, MOTOR_ACC, 1);
				delay_ms(5);
				Emm_V5_Vel_Control(motorAddr[2], 0, 0, MOTOR_ACC, 1);
				delay_ms(5);
				Emm_V5_Vel_Control(motorAddr[3], 0, 0, MOTOR_ACC, 1);
				delay_ms(5);
				Emm_V5_Synchronous_motion(0);
			}
			else
			{
				zeroStopTimer += MAIN_LOOP_DELAY_MS;  // 每轮累加计时
			}

			// 计时到 → 彻底锁死
			if(zeroStopTimer >= JOYSTICK_ZERO_STOP_DELAY_MS)
			{
				Motor_AllStop(motorAddr);
			}
		}
		else
		{
			// 摇杆有值 → 取消计时，正常解算
			zeroStopArmed = 0;
			zeroStopTimer = 0;

			// 5. 麦克纳姆轮运动学解算
			v1 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed + rotateSpeed)); // 左前
			v2 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed - rotateSpeed)); // 右前
			v3 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed + rotateSpeed)); // 左后
			v4 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed - rotateSpeed)); // 右后

			// 6. 下发速度指令到电机缓存
			Motor_SetSignedSpeed(motorAddr[0], MOTOR_LEFT_FORWARD_DIR, v1);
			delay_ms(5);
			Motor_SetSignedSpeed(motorAddr[1], MOTOR_RIGHT_FORWARD_DIR, v2);
			delay_ms(5);
			Motor_SetSignedSpeed(motorAddr[2], MOTOR_LEFT_FORWARD_DIR, v3);
			delay_ms(5);
			Motor_SetSignedSpeed(motorAddr[3], MOTOR_RIGHT_FORWARD_DIR, v4);
			delay_ms(5);
			
			// 7. 发送同步命令，四个电机同时执行
			Emm_V5_Synchronous_motion(0);
		}

		// 8. 舵机摆动 + 滑轨电机5运动（L1 + L2 同时触发）
		if((joystick.btn2 & (PS2_BTN_L1 | PS2_BTN_L2)) == (PS2_BTN_L1 | PS2_BTN_L2))
		{
			// 标记舵机正在运行（持续摆动）
			if(servoRunning == 0)
			{
				servoRunning = 1;
			}

			// 滑轨电机5仅在L1+L2按下的瞬间触发一次（边沿检测）
			if(!(prevBtn2 & PS2_BTN_L1) || !(prevBtn2 & PS2_BTN_L2))
			{
				if(slideHomed && !slideTriggered)
				{
					// 绝对位置运动至10cm处（逆时针方向）
					Emm_V5_Pos_Control(SLIDE_ADDR, 1, SLIDE_POS_VEL,
					                   SLIDE_POS_ACC, SLIDE_10CM_PULSES, 1, 0);
					delay_ms(MOTOR_CMD_DELAY_MS);
					Emm_V5_Synchronous_motion(0);
					slideTriggered = 1;
				}
			}

			delay_ms(50);

			Servo_Swing_Step(&servoPWM1, &servoDir1,
			                 &servoPWM2, &servoDir2,
			                 &servoPWM3, &servoDir3);
		}
		else
		{
			// L1+L2松开 → 停止舵机，重置滑轨触发标志
			if(servoRunning == 1)
			{
				servoRunning = 0;
			}
			if(slideTriggered)
			{
				slideTriggered = 0;
			}
		}

		prevBtn2 = joystick.btn2;  // 保存本次按键状态用于边沿检测

		delay_ms(MAIN_LOOP_DELAY_MS);
	}
}