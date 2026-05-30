#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f10x.h"

/**********************************************************
 * !其他
 **********************************************************/
#define MAIN_LOOP_DELAY_MS               20    /* 主循环每次执行后的延时，单位ms */

/**********************************************************
 * PS2手柄相关的参数
 **********************************************************/

/* PS2 joystick speed mapping */
#define JOYSTICK_CENTER                 128    /* PS2摇杆中位ADC值，偏离该值表示摇杆方向输入 */
#define JOYSTICK_DEAD_ZONE               12    /* 摇杆死区范围，小于该偏移量时认为没有速度输入 */
#define JOYSTICK_ZERO_STOP_DELAY_MS     500    /* 摇杆归零后等待此毫秒数再急停，让车利用惯性滑行 */
#define RJOYSTICK_MAX_SPEED             100    /* 右摇杆最大速度限制，限制旋转速度最大值 */

/**********************************************************
 * !电机相关的参数
 **********************************************************/

/* 电机地址*/
#define MOTOR_COUNT                       5    /* 小车使用的电机数量（包含第5号滑轨电机） */
#define MOTOR_ADDR_1                      1    /* 第1个电机驱动器地址 */
#define MOTOR_ADDR_2                      2    /* 第2个电机驱动器地址 */
#define MOTOR_ADDR_3                      3    /* 第3个电机驱动器地址 */
#define MOTOR_ADDR_4                      4    /* 第4个电机驱动器地址 */
#define MOTOR_ADDR_5                      5    /* 【已新增】第5个滑轨电机驱动器地址 */

/* 轮边电机的速度模式参数*/
#define MOTOR_MAX_RPM                   200   /* 电机最大目标转速，单位RPM，用于限制摇杆映射后的速度 */
#define MOTOR_ACC                       255    /* Emm_V5速度模式加速度参数，数值越大加减速越快 */

#define MOTOR_LEFT_FORWARD_DIR            0    /* 左侧电机前进时发送给驱动器的方向值 */
#define MOTOR_RIGHT_FORWARD_DIR           1    /* 右侧电机前进时发送给驱动器的方向值 */

/* Delay parameters, unit: ms */
#define SYSTEM_START_DELAY_MS          2000    /* 上电后等待电机驱动器初始化完成的延时，单位ms */
#define MOTOR_CMD_DELAY_MS                5    /* 连续发送电机控制命令之间的间隔，单位ms */

/* 
 * 5号滑轨电机相关的参数
 * Slide Rail Motor (Motor 5) - Position Control Parameters
 * Uses Emm_V5_Pos_Control with absolute mode (raF=1)
 */
/* Slide Rail Mechanism Parameters */
#define SLIDE_BELT_PITCH                2    /* 皮带节距为 2mm (标准GT2皮带) */
#define SLIDE_PULLEY_TEETH             20    /* 同步带轮为20齿 */

/* 自动计算：滑轨转一圈走多少毫米 (导程) */
#define SLIDE_LEAD      (SLIDE_PULLEY_TEETH * SLIDE_BELT_PITCH)
#define SLIDE_ADDR                    5               /* 滑轨电机驱动器地址 */

/* 回零参数（多圈无限位碰撞回零） */
#define SLIDE_HOMING_VEL              200             /* 回零速度，单位RPM */
#define SLIDE_HOMING_TIMEOUT_MS      5000             /* 回零等待超时，单位ms */
#define SLIDE_SL_VEL                  50              /* 碰撞检测转速，单位RPM */
#define SLIDE_SL_MA                   500             /* 碰撞检测电流，单位mA */
#define SLIDE_SL_MS                   500             /* 碰撞检测维持时间，单位ms */

/* 滑轨电机位置模式运动参数 */
#define SLIDE_POS_VEL                 300             /* 位置模式速度，单位RPM */
#define SLIDE_POS_ACC                 200              /* 位置模式加速度 */

/* 位置换算
 * 滑轨导程 = 25齿 × 2mm = 50mm/转
 * 1.8°步进电机，实测脉冲当量为 80 脉冲/mm
 * 10cm = 100mm × 80 = 8000 脉冲 */
#define SLIDE_PULSES_PER_MM           80              /* 80 脉冲/mm → 10cm = 8000 脉冲 */
#define SLIDE_10CM_PULSES            (100UL * SLIDE_PULSES_PER_MM)  /* 10cm对应脉冲数 */
#define SLIDE_5CM_PULSES             (50UL  * SLIDE_PULSES_PER_MM)  /* 5cm对应脉冲数 (50mm × 80 = 4000) */

/* ============================================================
 *  滑轨4个绝对位置（脉冲数，从零点开始）
 *  上电回零后自动上升到过渡位15cm
 * ============================================================ */
#define SLIDE_POS_TRANSIT_MM          150             /* 过渡位 15cm */
#define SLIDE_POS_TRANSIT            (SLIDE_POS_TRANSIT_MM * SLIDE_PULSES_PER_MM)  /* = 12000脉冲 */
#define SLIDE_POS_TRAY_PLACE_MM       130             /* 托盘放置位 13cm */
#define SLIDE_POS_TRAY_PLACE         (SLIDE_POS_TRAY_PLACE_MM * SLIDE_PULSES_PER_MM)  /* = 10400脉冲 */
#define SLIDE_POS_GRAB_MM             100             /* 物料夹取位 10cm */
#define SLIDE_POS_GRAB               (SLIDE_POS_GRAB_MM * SLIDE_PULSES_PER_MM)     /* = 8000脉冲 */
#define SLIDE_POS_PLACE_MM            70              /* 物料放置位 7cm */
#define SLIDE_POS_PLACE              (SLIDE_POS_PLACE_MM * SLIDE_PULSES_PER_MM)    /* = 5600脉冲 */

/* 从过渡位到各位置的相对移动脉冲数（+CCW上升，-CW下降） */
#define SLIDE_TRANSIT_TO_GRAB         ((int32_t)SLIDE_POS_GRAB   - (int32_t)SLIDE_POS_TRANSIT)  /* -4000（下降5cm） */
#define SLIDE_GRAB_TO_TRANSIT         ((int32_t)SLIDE_POS_TRANSIT - (int32_t)SLIDE_POS_GRAB)    /* +4000（上升5cm） */
#define SLIDE_TRANSIT_TO_PLACE        ((int32_t)SLIDE_POS_PLACE   - (int32_t)SLIDE_POS_TRANSIT) /* -6400（下降8cm） */
#define SLIDE_PLACE_TO_TRANSIT        ((int32_t)SLIDE_POS_TRANSIT - (int32_t)SLIDE_POS_PLACE)   /* +6400（上升8cm） */
#define SLIDE_TRANSIT_TO_TRAY_PLACE   ((int32_t)SLIDE_POS_TRAY_PLACE - (int32_t)SLIDE_POS_TRANSIT) /* -1600（下降2cm） */
#define SLIDE_TRAY_PLACE_TO_TRANSIT   ((int32_t)SLIDE_POS_TRANSIT - (int32_t)SLIDE_POS_TRAY_PLACE) /* +1600（上升2cm） */

/* 滑轨电机运动等待超时，单位ms */
#define SLIDE_MOVE_WAIT_MS           2500            /* 单次运动等待完成超时 */
#endif /* __CONFIG_H */



/**********************************************************
 * !舵机相关的参数
 * TIM8 uses APB2, CH1/CH2/CH3 on PC6/PC7/PC8
 **********************************************************/
#define SERVO_TIM                    TIM8            /* 使用TIM8高级定时器 */
#define SERVO_TIM_RCC                RCC_APB2Periph_TIM8  /* TIM8时钟在APB2上 */
#define SERVO_TIM_GPIO_RCC           RCC_APB2Periph_GPIOC /* 引脚GPIOC时钟 */

/* 舵机PWM引脚定义（TIM8通道）
 * PC6 = TIM8_CH1 = 夹爪（180°舵机）
 * PC7 = TIM8_CH2 = 云台（270°舵机）
 * PC8 = TIM8_CH3 = 托盘（270°舵机） */
#define SERVO_CLAW_PIN               GPIO_Pin_6      /* PC6 = TIM8_CH1 = 夹爪 */
#define SERVO_PTZ_PIN                GPIO_Pin_7      /* PC7 = TIM8_CH2 = 云台 */
#define SERVO_TRAY_PIN               GPIO_Pin_8      /* PC8 = TIM8_CH3 = 托盘 */
#define SERVO_GPIO                   GPIOC           /* 舵机PWM引脚所在端口 */

/* 舵机通用PWM脉冲宽度（50Hz=20ms，72000000/144=500KHz计数，ARR=9999）
 * 标准舵机 0.5ms~2.5ms 对应 0°~180°/270°（脉宽相同，角度范围不同） */
#define SERVO_PWM_MIN                250             /* 0.5ms 对应最小角度 */
#define SERVO_PWM_MID                750             /* 1.5ms 对应中位 */
#define SERVO_PWM_MAX                1250            /* 2.5ms 对应最大角度 */
#define SERVO_PWM_STEP               50              /* 每次摆动步进值 */
#define SERVO_ARR                    9999            /* 自动重装载值 */
#define SERVO_PSC                    143             /* 预分频值 */

/* 夹爪（PC6，180°舵机）角度与PWM映射 */
#define CLAW_PWM_MIN                250             /* 0.5ms = 0° */
#define CLAW_PWM_MID                750             /* 1.5ms = 90° */
#define CLAW_PWM_MAX                1250            /* 2.5ms = 180° */

/* 云台（PC7，270°舵机）角度与PWM映射 */
#define PTZ_PWM_MIN                 250             /* 0.5ms = 0° */
#define PTZ_PWM_MID                 750             /* 1.5ms = 135° */
#define PTZ_PWM_MAX                 1250            /* 2.5ms = 270° */

/* 托盘（PC8，270°舵机）角度与PWM映射 */
#define TRAY_PWM_MIN                250             /* 0.5ms = 0° */
#define TRAY_PWM_MID                750             /* 1.5ms = 135° */
#define TRAY_PWM_MAX                1250            /* 2.5ms = 270° */

/* ============================================================
 *  舵机位置定义（角度→PWM 换算）
 *  180°舵机（夹爪）:  每度 = 1000/180 ≈ 5.556 counts
 *  270°舵机（云台/托盘）: 每度 = 1000/270 ≈ 3.704 counts
 *  PWM = 250 + 角度 × 每度计数
 * ============================================================ */

/* ----- 夹爪（PC6, 180°舵机）位置 ----- */
#define CLAW_POS_GRIP             250             /* 夹取位（0°） */
#define CLAW_POS_RELEASE          583             /* 放料位（60°） */

/* ----- 舵机初始化位置 ----- */
#define CLAW_INIT_POS             250             /* 夹爪初始位置（0°） */
#define PTZ_INIT_POS              917             /* 云台初始位置（180°） */
#define TRAY_INIT_POS             417             /* 托盘初始位置（45°） */

/* ----- 云台（PC7, 270°舵机）位置 ----- */
#define PTZ_POS_GRIP              250             /* 夹取位（0°） */
#define PTZ_POS_TRAY1             694             /* 托盘位1（120°） */
#define PTZ_POS_TRAY2             917             /* 托盘位2（180°） */
#define PTZ_POS_TRAY3            1139             /* 托盘位3（240°） */

/* ----- 托盘（PC8, 270°舵机）位置 ----- */
#define TRAY_POS_1                694             /* 1号位（120°） */
#define TRAY_POS_2                917             /* 2号位（180°） */
#define TRAY_POS_3               1139             /* 3号位（240°） */
#define TRAY_POS_4               1250             /* 4号位（270°） */

/**********************************************************
 * !与电路相关的参数
 **********************************************************/

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


