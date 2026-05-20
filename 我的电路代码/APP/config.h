#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f10x.h"

/**********************************************************
 * Project configurable parameters
 * Change the values in this file when tuning the car.
 **********************************************************/

/* PS2 joystick speed mapping */
#define JOYSTICK_CENTER                 128    /* PS2摇杆中位ADC值，偏离该值表示摇杆方向输入 */
#define JOYSTICK_DEAD_ZONE               12    /* 摇杆死区范围，小于该偏移量时认为没有速度输入 */
#define JOYSTICK_ZERO_STOP_DELAY_MS     500    /* 摇杆归零后等待此毫秒数再急停，让车利用惯性滑行 */
#define RJOYSTICK_MAX_SPEED             100    /* 右摇杆最大速度限制，限制旋转速度最大值 */

/* Motor control */
#define MOTOR_COUNT                       5    /* 小车使用的电机数量（包含第5号滑轨电机） */
#define MOTOR_MAX_RPM                   200   /* 电机最大目标转速，单位RPM，用于限制摇杆映射后的速度 */
#define MOTOR_ACC                       255    /* Emm_V5速度模式加速度参数，数值越大加减速越快 */

#define MOTOR_ADDR_1                      1    /* 第1个电机驱动器地址 */
#define MOTOR_ADDR_2                      2    /* 第2个电机驱动器地址 */
#define MOTOR_ADDR_3                      3    /* 第3个电机驱动器地址 */
#define MOTOR_ADDR_4                      4    /* 第4个电机驱动器地址 */
#define MOTOR_ADDR_5                      5    /* 【已新增】第5个滑轨电机驱动器地址 */

#define MOTOR_LEFT_FORWARD_DIR            0    /* 左侧电机前进时发送给驱动器的方向值 */
#define MOTOR_RIGHT_FORWARD_DIR           1    /* 右侧电机前进时发送给驱动器的方向值 */

/* Delay parameters, unit: ms */
#define SYSTEM_START_DELAY_MS          2000    /* 上电后等待电机驱动器初始化完成的延时，单位ms */
#define MAIN_LOOP_DELAY_MS               20    /* 主循环每次执行后的延时，单位ms */
#define MOTOR_CMD_DELAY_MS                5    /* 连续发送电机控制命令之间的间隔，单位ms */

/* USART for Emm_V5 motor driver */
#define MOTOR_USART                    USART1  /* 与Emm_V5电机驱动器通信使用的串口外设 */
#define MOTOR_USART_IRQn               USART1_IRQn /* 电机串口对应的中断通道 */
#define MOTOR_USART_RCC                RCC_APB2Periph_USART1 /* 电机串口外设时钟 */
#define MOTOR_USART_GPIO               GPIOA   /* 电机串口TX/RX所在的GPIO端口 */
#define MOTOR_USART_GPIO_RCC           RCC_APB2Periph_GPIOA /* 电机串口GPIO端口时钟 */
#define MOTOR_USART_TX_PIN             GPIO_Pin_9 /* 电机串口发送引脚 */
#define MOTOR_USART_RX_PIN             GPIO_Pin_10 /* 电机串口接收引脚 */
#define MOTOR_USART_BAUDRATE           115200  /* 电机串口通信波特率 */

/* PS2 receiver pins */
#define PS2_GPIO                       GPIOA   /* PS2接收器信号线所在的GPIO端口 */
#define PS2_RCC_GPIO                   RCC_APB2Periph_GPIOA /* PS2接收器GPIO端口时钟 */
#define PS2_DI_PIN                     GPIO_Pin_4 /* PS2数据输入DI引脚，单片机从接收器读取数据 */
#define PS2_CMD_PIN                    GPIO_Pin_5 /* PS2命令输出CMD引脚，单片机向接收器发送命令 */
#define PS2_CS_PIN                     GPIO_Pin_6 /* PS2片选CS引脚，拉低时开始通信 */
#define PS2_CLK_PIN                    GPIO_Pin_7 /* PS2时钟CLK引脚，用于模拟通信时序 */
#define PS2_BIT_DELAY_US                 16    /* PS2每一位读写之间的延时，单位us */

/* (SERVO_TIM8 section deleted - TIM2 swing servos now in DRIVERS/servo_pwm.h) */

/* Slide Rail Mechanism Parameters */
#define SLIDE_BELT_PITCH                2    /* 皮带节距为 2mm (标准GT2皮带) */
#define SLIDE_PULLEY_TEETH             25    /* 【已修正】同步带轮为25齿 */

/* 自动计算：滑轨转一圈走多少毫米 (导程) */
#define SLIDE_LEAD      (SLIDE_PULLEY_TEETH * SLIDE_BELT_PITCH)

/**********************************************************
 * Servo (TIM8 advanced timer) parameters
 * TIM8 uses APB2, CH1/CH2/CH3 on PC6/PC7/PC8
 **********************************************************/
#define SERVO_TIM                    TIM8            /* 使用TIM8高级定时器 */
#define SERVO_TIM_RCC                RCC_APB2Periph_TIM8  /* TIM8时钟在APB2上 */
#define SERVO_TIM_GPIO_RCC           RCC_APB2Periph_GPIOC /* 引脚GPIOC时钟 */

/* 舵机PWM引脚定义（TIM8通道） */
#define SERVO1_PIN                   GPIO_Pin_6      /* PC6 = TIM8_CH1 */
#define SERVO2_PIN                   GPIO_Pin_7      /* PC7 = TIM8_CH2 */
#define SERVO3_PIN                   GPIO_Pin_8      /* PC8 = TIM8_CH3 */
#define SERVO_GPIO                   GPIOC           /* 舵机PWM引脚所在端口 */

/* 舵机角度与PWM脉冲宽度（50Hz=20ms，72000000/144=500KHz计数，ARR=9999） */
#define SERVO_PWM_MIN                250             /* 0.5ms 对应约0度 */
#define SERVO_PWM_MID                750             /* 1.5ms 对应约90度 */
#define SERVO_PWM_MAX                1250            /* 2.5ms 对应约180度 */
#define SERVO_PWM_STEP               50              /* 每次摆动步进值 */
#define SERVO_ARR                    9999            /* 自动重装载值 */
#define SERVO_PSC                    143             /* 预分频值 */

/**********************************************************
 * Slide Rail Motor (Motor 5) - Position Control Parameters
 * Uses Emm_V5_Pos_Control with absolute mode (raF=1)
 **********************************************************/
#define SLIDE_ADDR                    5               /* 滑轨电机驱动器地址 */

/* 回零参数（多圈无限位碰撞回零） */
#define SLIDE_HOMING_VEL              200             /* 回零速度，单位RPM */
#define SLIDE_HOMING_TIMEOUT_MS      5000             /* 回零等待超时，单位ms */
#define SLIDE_SL_VEL                  50              /* 碰撞检测转速，单位RPM */
#define SLIDE_SL_MA                   500             /* 碰撞检测电流，单位mA */
#define SLIDE_SL_MS                   500             /* 碰撞检测维持时间，单位ms */

/* 位置模式运动参数 */
#define SLIDE_POS_VEL                 300             /* 位置模式速度，单位RPM */
#define SLIDE_POS_ACC                 100              /* 位置模式加速度 */

/* 位置换算
 * 滑轨导程 = 25齿 × 2mm = 50mm/转
 * 1.8°步进电机，实测脉冲当量为 80 脉冲/mm
 * 10cm = 100mm × 80 = 8000 脉冲 */
#define SLIDE_PULSES_PER_MM           80              /* 80 脉冲/mm → 10cm = 8000 脉冲 */
#define SLIDE_10CM_PULSES            (100UL * SLIDE_PULSES_PER_MM)  /* 10cm对应脉冲数 */

#endif /* __CONFIG_H */
