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
    Emm_V5_Vel_Control(addr, dir, vel, MOTOR_ACC, 1);
}

/**
 * @brief  批量发送4个轮边电机的速度指令
 */
static void Motor_SetSpeed_Batch(const uint8_t motorAddr[4],
                                 int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
    Motor_SetSignedSpeed(motorAddr[0], MOTOR_LEFT_FORWARD_DIR,  v1);
    delay_ms(5);
    Motor_SetSignedSpeed(motorAddr[1], MOTOR_RIGHT_FORWARD_DIR, v2);
    delay_ms(5);
    Motor_SetSignedSpeed(motorAddr[2], MOTOR_LEFT_FORWARD_DIR,  v3);
    delay_ms(5);
    Motor_SetSignedSpeed(motorAddr[3], MOTOR_RIGHT_FORWARD_DIR, v4);
    delay_ms(5);

    Emm_V5_Synchronous_motion_All();
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
    }
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
 * @brief  设置带方向的相对位置移动
 */
static void Motor_SetSignedPosition(uint8_t addr, uint8_t forwardDir, int8_t direction)
{
    uint8_t  dir    = forwardDir;
    uint32_t pulses = CAMERA_ALIGN_STEP_PULSES;

    if(direction > 0)
    {
        pulses *= (uint32_t)direction;
    }
    else if(direction < 0)
    {
        dir     = (uint8_t)!forwardDir;
        pulses *= (uint32_t)(-direction);
    }
    else
    {
        pulses = 0;
    }

    Emm_V5_Pos_Control(addr, dir, CAMERA_ALIGN_POS_VEL, CAMERA_ALIGN_POS_ACC, pulses, 0, 1);
}

/**
 * @brief  批量发送4个轮边电机的位置指令（摄像头对准用）
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

    Emm_V5_Synchronous_motion_All();
}

/**
 * @brief  将摄像头视觉偏差值转换为离散的三级步长系数
 * 远距离：大步逼近(2cm)；中距离：中步靠拢(1cm)；近距离：精细微调(0.5cm)
 */
static int8_t Camera_ErrorToDirection(int8_t error)
{
    int8_t abs_err = (error > 0) ? error : -error;
    int8_t sign = (error > 0) ? 1 : -1;
    
    /* 离散三级步长量化器 */
    if(abs_err > 40)        // 距离较远：传出大阶梯系数 4
    {
        return sign * 4;
    }
    else if(abs_err > 15)   // 中等距离：传出中阶梯系数 2
    {
        return sign * 2;
    }
    else if(abs_err > 3)    // 接近中心：传出小阶梯微调系数 1
    {
        return sign * 1;
    }
    else                    // 进入死区
    {
        return 0;
    }
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
    CameraFrame_t cameraFrame;          
    uint8_t cameraStopCount = 0;         
    uint16_t cameraFrameTimeout = 0;     

    uint8_t  cameraMovingActive = 0;
    uint16_t cameraMoveWaitTimer = 0;
    int8_t   cameraForwardDir = 0;
    int8_t   cameraStrafeDir  = 0;

    /* 摇杆归零急停计时器 */
    uint16_t zeroStopTimer = 0;
    uint8_t  zeroStopArmed = 0;   
    uint8_t  zeroStopLocked = 0;  

    /* ----- 边沿检测 & 包执行器 ----- */
    uint8_t  prevBtn1 = 0;        
    uint8_t  prevBtn2 = 0;        

    // 硬件初始化
    board_init();
    Servo_PWM_Init();            
    
    // 上电设置舵机初始位置
    Servo_Claw_SetPulse(CLAW_INIT_POS);
    Servo_PTZ_SetPulse(PTZ_INIT_POS);
    Servo_Tray_SetPulse(TRAY_INIT_POS);
    
    PS2_Init();
    Package_Init();               

    // 初始状态强制停止
    for(i = 0; i < 4; i++)
    {
        Emm_V5_Stop_Now(motorAddr[i], 0);
        delay_ms(5);
    }

    delay_ms(2000);

    // 确保滑轨电机静止
    Emm_V5_Stop_Now(SLIDE_ADDR, 0);
    delay_ms(MOTOR_CMD_DELAY_MS);

    /* ===== 滑轨电机5回零参数配置 ===== */
    Emm_V5_Origin_Modify_Params(SLIDE_ADDR, 1, 2, 0, SLIDE_HOMING_VEL, SLIDE_HOMING_TIMEOUT_MS, SLIDE_SL_VEL, SLIDE_SL_MA, SLIDE_SL_MS, 1);
    delay_ms(100);

    /* 等待驱动器上电自动回零完成 */
    delay_ms(SLIDE_HOMING_TIMEOUT_MS);

    /* ===== 滑轨电机5回零完成后运动到过渡位 ===== */
    Emm_V5_Pos_Control(SLIDE_ADDR, 1, SLIDE_POS_VEL, SLIDE_POS_ACC, SLIDE_POS_TRANSIT, 1, 0);  

    delay_ms(SLIDE_MOVE_WAIT_MS);  

    while(1)
    {
        PS2_ScanKey(&joystick);

        // 0. 模式门控
        if(joystick.mode != 0x73)
        {
            AllStop(motorAddr);
            Package_Stop();          
            ps2ControlEnabled = 0;
            zeroStopArmed = 0;   
            cameraMovingActive = 0;  
            delay_ms(20);
            continue;
        }

        // 1. 安全急停逻辑
        if((joystick.btn2 & (PS2_BTN_R1 | PS2_BTN_R2)) == (PS2_BTN_R1 | PS2_BTN_R2))
        {
            AllStop(motorAddr);
            Package_Stop();          
            ps2ControlEnabled = 0;
            zeroStopArmed = 0;
            cameraMovingActive = 0;  
            delay_ms(20);
            continue;
        }

        // 2. 解锁逻辑
        if(ps2ControlEnabled == 0)
        {
            Motor_AllStop(motorAddr);
            if(joystick.btn1 & PS2_BTN_START)
            {
                Servo_PTZ_SetPulse(PTZ_POS_GRIP);
                Servo_Tray_SetPulse(TRAY_PWM_MIN);
                Servo_Claw_SetPulse(CLAW_POS_RELEASE);
                delay_ms(500);  
                ps2ControlEnabled = 1;
            }
            zeroStopArmed = 0;
            delay_ms(20);
            continue;
        }

        /* ============================================================
         * 最新修补版：摄像头视觉对准模块（并行接收 + 阶梯三级步长 + 状态机驱动）
         * ============================================================ */
        if(Package_IsCameraAlignPending())
        {
            /* 进入摄像头对准模式时，强制清除并释放摇杆锁定状态 */
            zeroStopArmed = 0;
            zeroStopTimer = 0;
            zeroStopLocked = 0; 

            /* 1. 【非阻塞运动超时检查】仅做状态恢复 */
            if(cameraMovingActive)
            {
                cameraMoveWaitTimer += MAIN_LOOP_DELAY_MS;
                if(cameraMoveWaitTimer >= CAMERA_ALIGN_MOVE_TIMEOUT_MS)
                {
                    Motor_AllStop(motorAddr);
                    cameraMovingActive = 0;
                    cameraMoveWaitTimer = 0;
                }
            }

            /* 2. 尝试获取一帧最新摄像头数据 */
            if(Camera_GetFrame(&cameraFrame))
            {
                cameraFrameTimeout = 0;

                /* ---- 分支 A：对准完成 (MODE_DONE = 0x02) ---- */
                if(cameraFrame.mode == 0x02) 
                {
                    if(cameraStopCount < CAMERA_STOP_CONFIRM_COUNT)
                    {
                        cameraStopCount++;
                    }

                    if(cameraStopCount >= CAMERA_STOP_CONFIRM_COUNT)
                    {
                        /* 先把小车停稳 */
                        Motor_AllStop(motorAddr); 
                        delay_ms(50);
                        
                        /* 🌟【物理动中断点】让小车原地剧烈抖动震下 200ms，宣告视觉交卷！ */
                        Motor_SetPosition_Batch(motorAddr, 15, -15, -15, 15); 
                        delay_ms(200);
                        Motor_AllStop(motorAddr); 
                        delay_ms(50);

                        /* 彻底粉碎摇杆死锁状态 */
                        zeroStopArmed = 0;
                        zeroStopTimer = 0;
                        zeroStopLocked = 0;

                        /* 状态变量彻底复位 */
                        Camera_ResetFrameState();               
                        cameraStopCount = 0;
                        cameraMovingActive = 0;
                        cameraMoveWaitTimer = 0;

                        /* 立即执行放置跳转 */
                        Package_CameraAlignDone();             /* 通知包执行器，继续后续放置动作 */
                    }
                }
                
                /* ---- 分支 B：正在追踪中 (MODE_TRACK = 0x01) ---- */
                else if(cameraFrame.mode == 0x01)
                {
                    cameraStopCount = 0; 

                    /* 如果上一次的步进动作还没执行完，先不覆盖新的速度命令，保护电机总线 */
                    if(!cameraMovingActive)
                    {
                        /* 此处已满血复活你的 0.5cm / 1cm / 2cm 阶梯三级步长控制 */
                        cameraForwardDir = Camera_ErrorToDirection(cameraFrame.dy); 
                        cameraStrafeDir  = Camera_ErrorToDirection(cameraFrame.dx); 

                        v1 = (int16_t)cameraForwardDir + (int16_t)cameraStrafeDir; 
                        v2 = (int16_t)cameraForwardDir - (int16_t)cameraStrafeDir; 
                        v3 = (int16_t)cameraForwardDir - (int16_t)cameraStrafeDir; 
                        v4 = (int16_t)cameraForwardDir + (int16_t)cameraStrafeDir; 

                        if(v1 > 127) v1 = 127; else if(v1 < -128) v1 = -128;
                        if(v2 > 127) v2 = 127; else if(v2 < -128) v2 = -128;
                        if(v3 > 127) v3 = 127; else if(v3 < -128) v3 = -128;
                        if(v4 > 127) v4 = 127; else if(v4 < -128) v4 = -128;

                        if(v1 != 0 || v2 != 0 || v3 != 0 || v4 != 0)
                        {
                            Motor_SetPosition_Batch(motorAddr, (int8_t)v1, (int8_t)v2, (int8_t)v3, (int8_t)v4);
                            cameraMovingActive = 1;
                            cameraMoveWaitTimer = 0;
                        }
                    }
                }
                
                /* ---- 分支 C：目标丢失 (MODE_LOST = 0x00) ---- */
                else if(cameraFrame.mode == 0x00)
                {
                    cameraStopCount = 0;
                    if(!cameraMovingActive)
                    {
                        Motor_AllStop(motorAddr); 
                    }
                }
            }
            /* 3. 获取帧失败 */
            else
            {
                if(cameraFrameTimeout < CAMERA_FRAME_TIMEOUT_MS)
                {
                    cameraFrameTimeout += MAIN_LOOP_DELAY_MS;
                }
                else
                {
                    Motor_AllStop(motorAddr);
                    Package_Stop();
                    Camera_ResetFrameState();
                    cameraStopCount = 0;
                    cameraFrameTimeout = 0;
                    cameraMovingActive = 0;
                    cameraMoveWaitTimer = 0;
                }
            }

            prevBtn1 = joystick.btn1;
            prevBtn2 = joystick.btn2;
            
            /* 🌟【核心修复点】即使在此处执行 continue 阻断，也必须强行调用一次状态机驱动！ */
            Package_Tick(); 
            
            delay_ms(MAIN_LOOP_DELAY_MS);
            continue; 
        }
        
        // 3. 读取手柄摇杆数据
        forwardSpeed = PS2_AxisToSpeed(joystick.LJoy_UD, 1, MOTOR_MAX_RPM);   
        strafeSpeed  = PS2_AxisToSpeed(joystick.LJoy_LR, 0, MOTOR_MAX_RPM);    
        rotateSpeed  = PS2_AxisToSpeed(joystick.RJoy_LR, 0, RJOYSTICK_MAX_SPEED); 

        // 4. 摇杆归零急停逻辑（非阻塞）
        if(forwardSpeed == 0 && strafeSpeed == 0 && rotateSpeed == 0)
        {
            if(zeroStopLocked)
            {
                /* 静默，什么都不发，避免干扰滑轨放置动作的串口命令 */
            }
            else if(zeroStopArmed == 0)
            {
                zeroStopArmed = 1;
                zeroStopTimer = 0;
                Motor_SetSpeed_Batch(motorAddr, 0, 0, 0, 0);
            }
            else
            {
                zeroStopTimer += MAIN_LOOP_DELAY_MS;  

                if(zeroStopTimer >= JOYSTICK_ZERO_STOP_DELAY_MS)
                {
                    Motor_AllStop(motorAddr);
                    zeroStopArmed  = 0;
                    zeroStopLocked = 1;   
                }
            }
        }
        else
        {
            zeroStopArmed  = 0;
            zeroStopTimer  = 0;
            zeroStopLocked = 0;

            // 5. 麦克纳姆轮运动学解算
            v1 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed + rotateSpeed)); 
            v2 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed - rotateSpeed)); 
            v3 = LimitMotorSpeed((int16_t)(forwardSpeed - strafeSpeed + rotateSpeed)); 
            v4 = LimitMotorSpeed((int16_t)(forwardSpeed + strafeSpeed - rotateSpeed)); 

            // 6. 批量下发速度指令
            Motor_SetSpeed_Batch(motorAddr, v1, v2, v3, v4);
        }

        // 7. 按键边沿触发 → 离散调用指定包

        /* 【十字键（btn1）→ 设置2号托盘目标角度】 */
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

        /* 【L1 + 图形键 → 装载托盘】 */
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
        }
        /* 【L2 + 图形键 → 卸载托盘】 */
        else if((joystick.btn2 & PS2_BTN_L2) && !(joystick.btn2 & PS2_BTN_L1))
        {
            if((joystick.btn2 & PS2_BTN_TRIANGLE) && !(prevBtn2 & PS2_BTN_TRIANGLE))
            {
                Camera_ResetFrameState();     
                cameraStopCount = 0;           
                cameraFrameTimeout = 0;         
                cameraMovingActive = 0;         
                Package_Start(UNLOAD_TRAY_1);
            }
            else if((joystick.btn2 & PS2_BTN_SQUARE) && !(prevBtn2 & PS2_BTN_SQUARE))
            {
                Camera_ResetFrameState();     
                cameraStopCount = 0;           
                cameraFrameTimeout = 0;         
                cameraMovingActive = 0;         
                Package_Start(UNLOAD_TRAY_2);
            }
            else if((joystick.btn2 & PS2_BTN_X) && !(prevBtn2 & PS2_BTN_X))
            {
                Camera_ResetFrameState();     
                cameraStopCount = 0;           
                cameraFrameTimeout = 0;         
                cameraMovingActive = 0;         
                Package_Start(UNLOAD_TRAY_3);
            }
        }

        // 8. 包执行器 Tick 驱动（常规手柄模式下驱动状态机）
        Package_Tick();

        prevBtn1 = joystick.btn1;
        prevBtn2 = joystick.btn2;  

        delay_ms(MAIN_LOOP_DELAY_MS);
    }
}