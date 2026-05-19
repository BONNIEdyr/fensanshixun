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

/* Motor control */
#define MOTOR_COUNT                       5    /* 小车使用的电机数量（包含第5号滑轨电机） */
#define MOTOR_MAX_RPM                   200   /* 电机最大目标转速，单位RPM，用于限制摇杆映射后的速度 */
#define MOTOR_ACC                       200    /* Emm_V5速度模式加速度参数，数值越大加减速越快 */

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
#define SLIDE_BELT_PITCH                2    /* 【已修正】皮带节距为 2mm (标准GT2皮带) */
#define SLIDE_PULLEY_TEETH             20    /* 【请根据实物修改】你的同步带轮（那个钢圈）有多少个齿 */

/* 自动计算：滑轨转一圈走多少毫米 (导程) */
#define SLIDE_LEAD      (SLIDE_PULLEY_TEETH * SLIDE_BELT_PITCH)
#endif /* __CONFIG_H */