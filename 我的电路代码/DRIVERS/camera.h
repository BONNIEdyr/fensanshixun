#ifndef __CAMERA_H
#define __CAMERA_H

#include "stm32f10x.h"

/* Camera UART: USART3, PB10=TX, PB11=RX */
#define CAMERA_USART                  USART3
#define CAMERA_USART_IRQn             USART3_IRQn
#define CAMERA_USART_RCC              RCC_APB1Periph_USART3
#define CAMERA_USART_GPIO             GPIOB
#define CAMERA_USART_GPIO_RCC         RCC_APB2Periph_GPIOB
#define CAMERA_USART_TX_PIN           GPIO_Pin_10
#define CAMERA_USART_RX_PIN           GPIO_Pin_11
#define CAMERA_USART_BAUDRATE         115200

/* Protocol: 0xAA + mode + dx(int8) + dy(int8) + 0x55 */
#define CAMERA_PROTOCOL_HEADER        0xAA
#define CAMERA_PROTOCOL_TAIL          0x55

#define CAMERA_MODE_NONE              0x00
#define CAMERA_MODE_OBJECT            0x01
#define CAMERA_MODE_RING              0x02

#define CAMERA_ALIGN_SPEED_RPM        60
#define CAMERA_STOP_CONFIRM_COUNT     3
#define CAMERA_FRAME_TIMEOUT_MS       300

typedef struct {
	uint8_t mode;
	int8_t dx;
	int8_t dy;
} CameraFrame_t;

void Camera_Init(void);
uint8_t Camera_GetFrame(CameraFrame_t *frame);
void Camera_ResetFrameState(void);

#endif /* __CAMERA_H */
