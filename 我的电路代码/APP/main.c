#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "ps2.h"
#include "config.h"
#include "servo_pwm.h"
#include "package_action.h"
#include "camera.h"            /* 摄像头视觉模块：通过USART3接收OpenMV发送的物体位置偏差数据 */

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
 * @brief  设置带方向的相对位置移动（带同步标志snF=1）
 * @param  addr       电机地址
 * @param  forwardDir 正向方向值（0或1）
 * @param  direction  移动方向和倍率：
 *                    0=不动, 1=正向2cm, -1=反向2cm, 2=正向4cm, -2=反向4cm
 * @note   进入同步等待缓存，需后续调用Emm_V5_Synchronous_motion_All触发
 *         使用相对位置模式 (raF=0)
 *         脉冲数 = |direction| × CAMERA_ALIGN_STEP_PULSES
 *         direction=0 时 pulses=0（电机不移动）
 */
static void Motor_SetSignedPosition(uint8_t addr, uint8_t forwardDir, int8_t direction)
{
	uint8_t  dir    = forwardDir;
	uint32_t pulses = CAMERA_ALIGN_STEP_PULSES;

	if(direction > 0)
	{
		/* 正向：方向不变，脉冲数 = direction × 单步步进 */
		pulses *= (uint32_t)direction;
	}
	else if(direction < 0)
	{
		/* 反向：方向取反，脉冲数 = |direction| × 单步步进 */
		dir     = (uint8_t)!forwardDir;
		pulses *= (uint32_t)(-direction);
	}
	else
	{
		/* direction == 0：不移动 */
		pulses = 0;
	}

	/* raF=0 相对位置模式, snF=1 进入同步缓存 */
	Emm_V5_Pos_Control(addr, dir, CAMERA_ALIGN_POS_VEL, CAMERA_ALIGN_POS_ACC,
	                   pulses, 0, 1);
}

/**
 * @brief  批量发送4个轮边电机的位置指令（摄像头对准用）
 * @note    每条指令之间加5ms延时，最后广播同步触发
 *         使用相对位置模式：每次发 CAMERA_ALIGN_STEP_PULSES（≈2cm）脉冲
 *         v1~v4 为方向值：1=正向, -1=反向, 0=不动
 */
static void Motor_SetPosition_Batch(const uint8_t motorAddr[4],
                                    int8_t dir1, int8_t dir2, int8_t dir3, int8_t dir4)
{
	Motor_SetSignedPosition(motorAddr[0], MOTOR_LEFT_FORWARD_DIR,  dir1);
	delay_ms(5);
	Motor_SetSignedPosition(motorAddr[1], MOTOR_RIGHT_FORWARD_DIR, dir2);
	delay_ms(5);
	Motor_SetSignedPosition(motorAddr[2], MOTOR_LEFT_FORWARD_DIR,  dir3);
	delay_ms(5);
	Motor_SetSignedPosition(motorAddr[3], MOTOR_RIGHT_FORWARD_DIR, dir4);
	delay_ms(5);

	/* 广播同步触发所有电机同时开始运动 */
	Emm_V5_Synchronous_motion_All();
}

/**
 * @brief  将摄像头视觉偏差值转换为移动方向
 * @param  error  摄像头检测到的位置偏差（dx或dy），int8_t范围
 * @retval 正偏差返回1（正向移动），负偏差返回-1（反向移动），0返回0（无需移动）
 * @note   摄像头对准改为位置模式：每次偏差非零时走固定2cm步进，
 *         走完后重新获取一帧判断，直至偏差为零
 */
static int8_t Camera_ErrorToDirection(int8_t error)
{
	if(error > 0) return 1;
	if(error < 0) return -1;
	return 0;
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
	CameraFrame_t cameraFrame;          /* 摄像头帧数据：包含识别模式(mode)和xy方向偏差(dx/dy) */
	uint8_t cameraStopCount = 0;         /* 摄像头停止确认计数器：连续CAMERA_STOP_CONFIRM_COUNT次检测到偏差为0才判定对准完成 */
	uint16_t cameraFrameTimeout = 0;     /* 摄像头帧超时计时器：连续收不到有效帧数据超过阈值时强制停止 */

	/*
	 * 摄像头对准位置模式状态变量：
	 *   cameraMovingActive： 1 = 已发出2cm位置指令，正在等待运动完成；0 = 空闲，可以获取下一帧
	 *   cameraMoveWaitTimer：从发出位置指令开始计时的超时计数器
	 *   cameraForwardDir：上次位置移动的forward方向（用于偏差为零分支前先等走完）
	 *   cameraStrafeDir： 上次位置移动的strafe方向
	 */
	uint8_t  cameraMovingActive = 0;
	uint16_t cameraMoveWaitTimer = 0;
	int8_t   cameraForwardDir = 0;
	int8_t   cameraStrafeDir  = 0;

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
			cameraMovingActive = 0;  /* 清除位置移动状态 */
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
			cameraMovingActive = 0;  /* 清除位置移动状态 */
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

		/* ============================================================
		 *  摄像头视觉对准模块（位置模式）
		 *  当包执行器调用Package_Start(UNLOAD_TRAY_x)后，云台转到对应托盘位
		 *  随后包执行器设置CameraAlignPending标志，主循环进入此分支
		 *
		 *  位置模式逻辑：
		 *    1. 获取一帧 → 判断偏差
		 *    2. 偏差非零 → 解算4轮方向 → 发一次2cm相对位置指令 → 等待走完
		 *    3. 走完后重新获取下一帧，重复直到偏差=0
		 *    4. 偏差=0连续CAMERA_STOP_CONFIRM_COUNT帧 → 对准完成
		 * ============================================================ */
		if(Package_IsCameraAlignPending())
		{
			/* 进入摄像头对准模式时，清除摇杆归零急停状态，避免干扰视觉对准 */
			zeroStopArmed = 0;
			zeroStopTimer = 0;

			/*
			 * 如果当前正在等待上一次位置移动完成：
			 *   - 检查超时
			 *   - 超时则强制停止并重置状态，重新获取帧
			 *   - 未超时则继续等待下一轮
			 */
			if(cameraMovingActive)
			{
				cameraMoveWaitTimer += MAIN_LOOP_DELAY_MS;
				if(cameraMoveWaitTimer >= CAMERA_ALIGN_MOVE_TIMEOUT_MS)
				{
					/* 超时：强制停止所有电机，重置状态，重新开始对准 */
					Motor_AllStop(motorAddr);
					cameraMovingActive = 0;
					cameraMoveWaitTimer = 0;
				}
				/* 未超时 → 继续等待运动完成，本轮跳过摄像头帧获取 */
				prevBtn1 = joystick.btn1;
				prevBtn2 = joystick.btn2;
				delay_ms(MAIN_LOOP_DELAY_MS);
				continue;
			}

			/* ----- 尝试获取一帧摄像头数据 ----- */
			if(Camera_GetFrame(&cameraFrame))
			{
				/* 成功获取到帧，重置超时计数器 */
				cameraFrameTimeout = 0;

				/*
				 * 判断条件：mode == NONE 且 dx==0 且 dy==0
				 * 表示OpenMV识别到物体已位于图像中心，或者未检测到目标物体
				 * （即偏差为零 → 已对准或无需调整）
				 */
				if(cameraFrame.mode == CAMERA_MODE_NONE && cameraFrame.dx == 0 && cameraFrame.dy == 0)
				{
					/*
					 * 防抖处理：连续CAMERA_STOP_CONFIRM_COUNT次
					 * 检测到偏差为0才判定为真正对准完成，避免偶发噪声误判
					 */
					if(cameraStopCount < CAMERA_STOP_CONFIRM_COUNT)
					{
						cameraStopCount++;
					}

					/* 达到确认次数 → 对准完成 */
					if(cameraStopCount >= CAMERA_STOP_CONFIRM_COUNT)
					{
						Motor_AllStop(motorAddr);                /* 急停锁定所有电机 */
						Package_CameraAlignDone();               /* 通知包执行器摄像头对准完成，继续后续动作 */
						Camera_ResetFrameState();                 /* 复位摄像头帧状态机，准备下一次识别 */
						cameraStopCount = 0;                      /* 复位停止计数器 */
						cameraMovingActive = 0;                   /* 复位位置移动状态 */
					}
				}
				else
				{
					/*
					 * 偏差不为零 → 需要调整位置
					 * 重置停止计数器，防止未对准时误判
					 */
					cameraStopCount = 0;

					/*
					 * 将偏差映射为移动方向（位置模式）：
					 *   forwardDir ← dy（前后方向：1=前进 / -1=后退 / 0=不动）
					 *   strafeDir  ← dx（左右方向：1=右移 / -1=左移 / 0=不动）
					 */
					cameraForwardDir = Camera_ErrorToDirection(cameraFrame.dy);
					cameraStrafeDir  = Camera_ErrorToDirection(cameraFrame.dx);

					/*
					 * 麦克纳姆轮方向解算（位置模式）：
					 * 方向映射与速度模式一致，但用方向值(±1)替代速度值
					 *   dir1(左前) = forwardDir + strafeDir
					 *   dir2(右前) = forwardDir - strafeDir
					 *   dir3(左后) = forwardDir - strafeDir
					 *   dir4(右后) = forwardDir + strafeDir
					 *
					 * 注意：方向值可能为 -2/-1/0/1/2，
					 *       ±2 表示该轮需走双倍距离（对角线移动）
					 */
					v1 = (int16_t)cameraForwardDir + (int16_t)cameraStrafeDir;
					v2 = (int16_t)cameraForwardDir - (int16_t)cameraStrafeDir;
					v3 = (int16_t)cameraForwardDir - (int16_t)cameraStrafeDir;
					v4 = (int16_t)cameraForwardDir + (int16_t)cameraStrafeDir;

					/* 限幅到 -128~+127（int8_t 范围），然后用 int8_t 传递方向值 */
					if(v1 > 127) v1 = 127; else if(v1 < -128) v1 = -128;
					if(v2 > 127) v2 = 127; else if(v2 < -128) v2 = -128;
					if(v3 > 127) v3 = 127; else if(v3 < -128) v3 = -128;
					if(v4 > 127) v4 = 127; else if(v4 < -128) v4 = -128;

					/* 批量下发相对位置指令（每次2cm）+ 广播同步触发 */
					Motor_SetPosition_Batch(motorAddr,
					                        (int8_t)v1, (int8_t)v2,
					                        (int8_t)v3, (int8_t)v4);

					/* 标记正在移动，开始超时计时 */
					cameraMovingActive = 1;
					cameraMoveWaitTimer = 0;
				}
			}
			else
			{
				/*
				 * 获取帧失败（如串口接收中断、帧校验错误等）
				 * 启动超时计数器：若持续CAMERA_FRAME_TIMEOUT_MS收不到有效帧，
				 * 则急停电机、退出摄像头对准模式，回到正常摇杆控制状态
				 */
				if(cameraFrameTimeout < CAMERA_FRAME_TIMEOUT_MS)
				{
					cameraFrameTimeout += MAIN_LOOP_DELAY_MS;
				}
				else
				{
					Motor_AllStop(motorAddr);          /* 急停所有电机 */
					Package_Stop();                     /* 退出摄像头对准模式 */
					Camera_ResetFrameState();           /* 复位摄像头帧状态机 */
					cameraStopCount = 0;                 /* 复位停止确认计数 */
					cameraFrameTimeout = 0;              /* 复位超时计时器 */
					cameraMovingActive = 0;              /* 复位位置移动等待状态 */
				}
			}

			prevBtn1 = joystick.btn1;
			prevBtn2 = joystick.btn2;
			delay_ms(MAIN_LOOP_DELAY_MS);
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

		/* 【十字键（btn1）→ 设置2号托盘目标角度】 */
		/* ↓ → 0° */
		if((joystick.btn1 & PS2_BTN_DOWN) && !(prevBtn1 & PS2_BTN_DOWN))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_0);
		}
		/* ← → 90° */
		else if((joystick.btn1 & PS2_BTN_LEFT) && !(prevBtn1 & PS2_BTN_LEFT))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_90);
		}
		/* ↑ → 180° */
		else if((joystick.btn1 & PS2_BTN_UP) && !(prevBtn1 & PS2_BTN_UP))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_180);
		}
		/* → → 270° */
		else if((joystick.btn1 & PS2_BTN_RIGHT) && !(prevBtn1 & PS2_BTN_RIGHT))
		{
			Set_Tray2_Target_Angle(TRAY_POS_DEG_270);
		}

		/* 【L1（左肩键1）+ 图形键 → 装载托盘】 */
		if((joystick.btn2 & PS2_BTN_L1) && !(joystick.btn2 & PS2_BTN_L2))
		{
			/* L1 + △ → 装载1号托盘 */
			if((joystick.btn2 & PS2_BTN_TRIANGLE) && !(prevBtn2 & PS2_BTN_TRIANGLE))
			{
				Package_Start(LOAD_TRAY_1);
			}
			/* L1 + □ → 装载2号托盘 */
			else if((joystick.btn2 & PS2_BTN_SQUARE) && !(prevBtn2 & PS2_BTN_SQUARE))
			{
				Package_Start(LOAD_TRAY_2);
			}
			/* L1 + × → 装载3号托盘 */
			else if((joystick.btn2 & PS2_BTN_X) && !(prevBtn2 & PS2_BTN_X))
			{
				Package_Start(LOAD_TRAY_3);
			}
		}
		/* 【L2（左肩键2）+ 图形键 → 卸载托盘（含摄像头状态复位）】 */
		else if((joystick.btn2 & PS2_BTN_L2) && !(joystick.btn2 & PS2_BTN_L1))
		{
			/* L2 + △ → 卸载1号托盘 */
			if((joystick.btn2 & PS2_BTN_TRIANGLE) && !(prevBtn2 & PS2_BTN_TRIANGLE))
			{
				/* 触发卸载1号托盘前，复位摄像头状态机、停止计数器和超时计数器 */
				Camera_ResetFrameState();     /* 复位摄像头帧接收状态机，清空内部缓冲区 */
				cameraStopCount = 0;           /* 复位停止确认计数，避免旧数据干扰 */
				cameraFrameTimeout = 0;         /* 复位超时计时器 */
				cameraMovingActive = 0;         /* 复位位置移动等待状态 */
				Package_Start(UNLOAD_TRAY_1);
			}
			/* L2 + □ → 卸载2号托盘 */
			else if((joystick.btn2 & PS2_BTN_SQUARE) && !(prevBtn2 & PS2_BTN_SQUARE))
			{
				/* 触发卸载2号托盘前，复位摄像头状态机、停止计数器和超时计数器 */
				Camera_ResetFrameState();     /* 复位摄像头帧接收状态机，清空内部缓冲区 */
				cameraStopCount = 0;           /* 复位停止确认计数，避免旧数据干扰 */
				cameraFrameTimeout = 0;         /* 复位超时计时器 */
				cameraMovingActive = 0;         /* 复位位置移动等待状态 */
				Package_Start(UNLOAD_TRAY_2);
			}
			/* L2 + × → 卸载3号托盘 */
			else if((joystick.btn2 & PS2_BTN_X) && !(prevBtn2 & PS2_BTN_X))
			{
				/* 触发卸载3号托盘前，复位摄像头状态机、停止计数器和超时计数器 */
				Camera_ResetFrameState();     /* 复位摄像头帧接收状态机，清空内部缓冲区 */
				cameraStopCount = 0;           /* 复位停止确认计数，避免旧数据干扰 */
				cameraFrameTimeout = 0;         /* 复位超时计时器 */
				cameraMovingActive = 0;         /* 复位位置移动等待状态 */
				Package_Start(UNLOAD_TRAY_3);
			}
		}

		// 8. 包执行器 Tick 驱动（在主循环中每轮调用）
		Package_Tick();

		prevBtn1 = joystick.btn1;
		prevBtn2 = joystick.btn2;  // 保存本次按键状态用于边沿检测

		delay_ms(MAIN_LOOP_DELAY_MS);
	}
}