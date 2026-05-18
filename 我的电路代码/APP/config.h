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
#define MOTOR_COUNT                       4    /* 小车使用的电机数量 */
#define MOTOR_MAX_RPM                   200   /* 电机最大目标转速，单位RPM，用于限制摇杆映射后的速度 */
#define MOTOR_ACC                       200    /* Emm_V5速度模式加速度参数，数值越大加减速越快 */

#define MOTOR_ADDR_1                      1    /* 第1个电机驱动器地址 */
#define MOTOR_ADDR_2                      2    /* 第2个电机驱动器地址 */
#define MOTOR_ADDR_3                      3    /* 第3个电机驱动器地址 */
#define MOTOR_ADDR_4                      4    /* 第4个电机驱动器地址 */

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

/* Servo PWM output */
#define SERVO_TIM                      TIM8    /* 产生舵机PWM信号使用的定时器 */
#define SERVO_TIM_RCC                  RCC_APB2Periph_TIM8 /* 舵机PWM定时器时钟 */
#define SERVO_GPIO                     GPIOC   /* 舵机PWM输出引脚所在的GPIO端口 */
#define SERVO_GPIO_RCC                 RCC_APB2Periph_GPIOC /* 舵机PWM GPIO端口时钟 */
#define SERVO_TRAY_PIN                 GPIO_Pin_6 /* 2号托盘舵机，白线，PC6，TIM8_CH1 */
#define SERVO_GIMBAL_PIN               GPIO_Pin_7 /* 1号云台舵机，灰线，PC7，TIM8_CH2 */
#define SERVO_CLAW_PIN                 GPIO_Pin_8 /* 3号夹爪舵机，橙线，PC8，TIM8_CH3 */

#define SERVO_CH1_PIN                  SERVO_TRAY_PIN /* TIM8通道1：托盘舵机 */
#define SERVO_CH2_PIN                  SERVO_GIMBAL_PIN /* TIM8通道2：云台舵机 */
#define SERVO_CH3_PIN                  SERVO_CLAW_PIN /* TIM8通道3：夹爪舵机 */

#define SERVO_TIM_PERIOD               (200 - 1) /* 舵机PWM定时器自动重装载值，决定PWM周期计数上限 */
#define SERVO_TIM_PRESCALER            (7200 - 1) /* 舵机PWM定时器预分频值，决定计数频率 */

#define SERVO_MIN_ANGLE                   0    /* 舵机最小角度 */
#define SERVO_MAX_ANGLE                 180    /* 舵机最大角度 */
#define SERVO_MIN_PULSE                   5    /* 0度对应PWM比较值，约0.5ms */
#define SERVO_ZERO_PULSE                 15    /* 90度零位对应PWM比较值，约1.5ms */
#define SERVO_MAX_PULSE                  25    /* 180度对应PWM比较值，约2.5ms */

#define SERVO_GIMBAL_ZERO_ANGLE           0    /* 云台舵机零位角度 */
#define SERVO_TRAY_ZERO_ANGLE             0    /* 托盘舵机零位角度 */
#define SERVO_CLAW_ZERO_ANGLE            90    /* 夹爪舵机零位角度 */

#define SERVO_CLAW_CLOSE_ANGLE          150    /* 夹爪动作角度：零位基础上正向转动60度 */
#define SERVO_GIMBAL_TURN_ANGLE         180    /* 云台动作角度：从零位转到180度 */
#define SERVO_TRAY_TURN_ANGLE            90    /* 托盘动作角度：从零位转到90度 */
#define SERVO_ACTION_DELAY_MS          2000    /* 舵机动作之间的等待时间，单位ms */

#endif
