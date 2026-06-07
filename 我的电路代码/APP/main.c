#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "ps2.h"
#include "config.h"
#include "servo_pwm.h"
#include "package_action.h"

/* 手柄右侧图形键 (对应 btn2) */
#define PS2_BTN_TRIANGLE 0x10
#define PS2_BTN_CIRCLE   0x20
#define PS2_BTN_X        0x40
#define PS2_BTN_SQUARE   0x80

/* 手柄左侧十字键 (对应 btn1) */
#define PS2_BTN_UP       0x10
#define PS2_BTN_RIGHT    0x20
#define PS2_BTN_DOWN     0x40
#define PS2_BTN_LEFT     0x80

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
 * @brief  设置带方向的速度（带同步标志snF=1）
 * @note   进入同步等待缓存，需后续调用Emm_V5_Synchronous_motion_All触发
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
	// snF=1，进入同步等待缓存
	Emm_V5_Vel_Control(addr, dir, vel, MOTOR_ACC, 1);
}

/**
 * @brief  批量发送4个轮边电机的速度指令（不插入延时）
 * @note   usart_SendByte本身阻塞等待TXE，字节自动按序发送
 *         snF=1使所有指令进入同步缓存，最后用一次广播触发
 */
static void Motor_SetSpeed_Batch(const uint8_t motorAddr[4],
                                 int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	/* 每条速度指令之间加5ms延时，确保电机驱动器处理完上一条指令 */
	Motor_SetSignedSpeed(motorAddr[0], MOTOR_LEFT_FORWARD_DIR,  v1);
	delay_ms(5);
	Motor_SetSignedSpeed(motorAddr[1], MOTOR_RIGHT_FORWARD_DIR, v2);
	delay_ms(5);
	Motor_SetSignedSpeed(motorAddr[2], MOTOR_LEFT_FORWARD_DIR,  v3);
	delay_ms(5);
	Motor_SetSignedSpeed(motorAddr[3], MOTOR_RIGHT_FORWARD_DIR, v4);
	delay_ms(5);

	/* 广播地址0同步触发：一条命令触发所有4个电机同时开始运动 */
	Emm_V5_Synchronous_motion_All();
}

/**
 * @brief  全机立即停止（发送停止命令 + 广播同步触发）
 */
static void Motor_AllStop(const uint8_t motorAddr[4])
{
	uint8_t i;
	for(i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 1);  /* snF=1 进入同步缓存 */
	}
	/* 广播同步触发，所有电机同时停止 */
	Emm_V5_Synchronous_motion_All();
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
	uint8_t  zeroStopLocked = 0;  // 1=已彻底锁死，保持静止不再重发命令


	/* ----- 边沿检测 & 包执行器 ----- */
	uint8_t  prevBtn1 = 0;        // 上一次的btn1值，用于按键边沿检测
	uint8_t  prevBtn2 = 0;        // 上一次的btn2值，用于按键边沿检测

	// 硬件初始化
	board_init();
	Servo_PWM_Init();            // 初始化TIM8舵机PWM（PC6_CH1/PC7_CH2/PC8_CH3）
	
	// 上电设置舵机初始位置：夹爪0°，云台180°，托盘45°
	Servo_Claw_SetPulse(CLAW_INIT_POS);
	Servo_PTZ_SetPulse(PTZ_INIT_POS);
	Servo_Tray_SetPulse(TRAY_INIT_POS);
	
	PS2_Init();
	Package_Init();               // 初始化包执行器

	// 初始状态强制停止
	for(i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(motorAddr[i], 0);
		delay_ms(5);
	}

	// 等待驱动器初始化
	delay_ms(2000);

	// 确保滑轨电机静止
	Emm_V5_Stop_Now(SLIDE_ADDR, 0);
	delay_ms(MOTOR_CMD_DELAY_MS);

	/* ===== 滑轨电机5回零参数配置（有记忆模式：上电自动回零） ===== */
	// svF=true 存储到驱动器Flash，掉电不丢失
	// o_mode=2 多圈无限位碰撞回零
	// o_dir=0  CW方向向下运动寻找零位
	// potF=true 上电自动触发回零（有记忆模式的核心）
	Emm_V5_Origin_Modify_Params(SLIDE_ADDR,
	                            1,                    // svF=true，存储到Flash
	                            2,                    // o_mode=2，多圈无限位碰撞回零
	                            0,                    // o_dir=0，CW方向向下碰零位
	                            SLIDE_HOMING_VEL,     // 回零速度
	                            SLIDE_HOMING_TIMEOUT_MS,  // 回零超时
	                            SLIDE_SL_VEL,         // 碰撞检测转速
	                            SLIDE_SL_MA,          // 碰撞检测电流
	                            SLIDE_SL_MS,          // 碰撞检测维持时间
	                            1);                   // potF=true，上电自动触发回零
	delay_ms(100);

	/* 等待驱动器上电自动回零完成（potF=true 已配置，驱动器上电自动执行回零） */
	delay_ms(SLIDE_HOMING_TIMEOUT_MS);

	/* ===== 滑轨电机5回零完成后运动到过渡位（绝对位置模式 raF=1） =====
	 * snF=0：单轴运动立即执行，不进入多机同步缓存。
	 * 若用 snF=1 会一直等广播同步命令才动，导致上电时滑轨可能不上升到过渡位。 */
	Emm_V5_Pos_Control(SLIDE_ADDR, 1, SLIDE_POS_VEL, SLIDE_POS_ACC,
	                   SLIDE_POS_TRANSIT, 1, 0);  // raF=1 绝对位置模式 dir=1向零位上方 → 上升到过渡位13cm，snF=0 立即执行

	delay_ms(SLIDE_MOVE_WAIT_MS);  // 等待运动完成（上电这里暂保留固定延时）

	while(1)
	{
		PS2_ScanKey(&joystick);

		// 0. 模式门控：只有绿灯模拟模式(0x73)才执行控制逻辑
		//    红灯模式或断连时停止电机并上锁
		if(joystick.mode != 0x73)
		{
			AllStop(motorAddr);
			Package_Stop();          // 包执行器也停止
			ps2ControlEnabled = 0;
			zeroStopArmed = 0;   // 清除急停计时
			delay_ms(20);
			continue;
		}

		// 1. 安全急停逻辑：同时按 R1 + R2
		if((joystick.btn2 & (PS2_BTN_R1 | PS2_BTN_R2)) == (PS2_BTN_R1 | PS2_BTN_R2))
		{
			AllStop(motorAddr);
			Package_Stop();          // 包执行器停止
			ps2ControlEnabled = 0;
			zeroStopArmed = 0;
			delay_ms(20);
			continue;
		}

		// 2. 解锁逻辑：按下 START 键 → 舵机复位准备
		if(ps2ControlEnabled == 0)
		{
			Motor_AllStop(motorAddr);
			if(joystick.btn1 & PS2_BTN_START)
			{
				// 云台从180°回到0°夹取位
				Servo_PTZ_SetPulse(PTZ_POS_GRIP);
				// 托盘从45°回到0°
				Servo_Tray_SetPulse(TRAY_PWM_MIN);
				// 夹爪从闭合位张开
				Servo_Claw_SetPulse(CLAW_POS_RELEASE);
				delay_ms(500);  // 等待舵机运动到位
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
		//
		//    【方案B：边移动边取放料】允许包动作执行期间同时遥控小车。
		//    USART1 由滑轨(地址5)和4个轮边电机共用，驱动器靠串口空闲(IDLE)分帧，
		//    因此在 Package_ExecuteStep 下发滑轨命令的前后各插入串口保护间隔，
		//    人为制造空闲期，避免与轮边命令撞帧而丢失滑轨命令。
		if(forwardSpeed == 0 && strafeSpeed == 0 && rotateSpeed == 0)


		{
			/* 已彻底锁死 → 保持静止，不再重复发送任何命令（避免无意义地
			 * 周期性占用共用串口，干扰滑轨命令）。直到摇杆再次有值才解除。 */
			if(zeroStopLocked)
			{
				/* 静默，什么都不发 */
			}
			// 首次进入 → 立即发送速度0，让电机按MOTOR_ACC减速
			else if(zeroStopArmed == 0)
			{
				zeroStopArmed = 1;
				zeroStopTimer = 0;

				// 立即给所有电机发速度0（带MOTOR_ACC，自然减速）
				// 使用批量发送+usart_SendByte阻塞等待TXE，无需delay_ms
				Motor_SetSpeed_Batch(motorAddr, 0, 0, 0, 0);
			}
			else
			{
				zeroStopTimer += MAIN_LOOP_DELAY_MS;  // 每轮累加计时

				// 计时到 → 彻底锁死一次，随后保持静止不再重发
				if(zeroStopTimer >= JOYSTICK_ZERO_STOP_DELAY_MS)
				{
					Motor_AllStop(motorAddr);
					zeroStopArmed  = 0;
					zeroStopLocked = 1;   // 置锁，后续保持静止
				}
			}
		}
		else
		{
			// 摇杆有值 → 取消计时与锁死，恢复正常控制
			zeroStopArmed  = 0;
			zeroStopTimer  = 0;
			zeroStopLocked = 0;


			// 5. 麦克纳姆轮运动学解算
			v1 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed + rotateSpeed)); // 左前
			v2 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed - rotateSpeed)); // 右前
			v3 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed + rotateSpeed)); // 左后
			v4 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed - rotateSpeed)); // 右后

			// 6. 批量下发速度指令到电机缓存 + 广播同步触发
			//    连续发送4条命令（usart_SendByte阻塞等待TXE，无需delay_ms），
			//    最后一条广播同步命令触发所有4个电机同时运动
			Motor_SetSpeed_Batch(motorAddr, v1, v2, v3, v4);
		}

		// 7. 按键边沿触发 → 离散调用指定包
		if((joystick.btn2 & PS2_BTN_L1) && !(joystick.btn2 & PS2_BTN_L2))
		{
			if((joystick.btn2 & PS2_BTN_TRIANGLE) && !(prevBtn2 & PS2_BTN_TRIANGLE))
			{
				Package_Start(LOAD_TRAY_1);
			}
			else if((joystick.btn2 & PS2_BTN_SQUARE) && !(prevBtn2 & PS2_BTN_SQUARE))
			{
				Package_Start(LOAD_TRAY_2);
			}
			else if((joystick.btn2 & PS2_BTN_X) && !(prevBtn2 & PS2_BTN_X))
			{
				Package_Start(LOAD_TRAY_3);
			}
			else if((joystick.btn2 & PS2_BTN_CIRCLE) && !(prevBtn2 & PS2_BTN_CIRCLE))
			{
				// 圆圈键：与 LOAD_TRAY_2 相同流程，但夹取时滑轨下降到物料放置位(零位)
				Package_Start(LOAD_TRAY_2_FROM_PLACE);
			}
		}
		else if((joystick.btn2 & PS2_BTN_L2) && !(joystick.btn2 & PS2_BTN_L1))
		{
			if((joystick.btn2 & PS2_BTN_TRIANGLE) && !(prevBtn2 & PS2_BTN_TRIANGLE))
			{
				Package_Start(UNLOAD_TRAY_1);
			}
			else if((joystick.btn2 & PS2_BTN_SQUARE) && !(prevBtn2 & PS2_BTN_SQUARE))
			{
				Package_Start(UNLOAD_TRAY_2);
			}
			else if((joystick.btn2 & PS2_BTN_X) && !(prevBtn2 & PS2_BTN_X))
			{
				Package_Start(UNLOAD_TRAY_3);
			}
		}

		if((joystick.btn1 & PS2_BTN_DOWN) && !(prevBtn1 & PS2_BTN_DOWN))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_0);
		}
		else if((joystick.btn1 & PS2_BTN_LEFT) && !(prevBtn1 & PS2_BTN_LEFT))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_90);
		}
		else if((joystick.btn1 & PS2_BTN_UP) && !(prevBtn1 & PS2_BTN_UP))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_180);
		}
		else if((joystick.btn1 & PS2_BTN_RIGHT) && !(prevBtn1 & PS2_BTN_RIGHT))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_270);
		}

		// 8. 包执行器 Tick 驱动（在主循环中每轮调用）
		Package_Tick();

		prevBtn1 = joystick.btn1;
		prevBtn2 = joystick.btn2;  // 保存本次按键状态用于边沿检测

		delay_ms(MAIN_LOOP_DELAY_MS);
	}
}


