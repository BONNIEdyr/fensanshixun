#include "camera.h"

static volatile CameraFrame_t g_cameraFrame;
static volatile uint8_t g_cameraFrameReady = 0;
static volatile uint8_t g_cameraRxBuf[5];
static volatile uint8_t g_cameraRxCount = 0;

/**
 * @brief  初始化摄像头串口。
 * @note   使用 USART3，PB10 为 TX，PB11 为 RX，波特率由 CAMERA_USART_BAUDRATE 指定。
 */
void Camera_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 使能 GPIO/AFIO 和 USART3 时钟。 */
	RCC_APB2PeriphClockCmd(CAMERA_USART_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(CAMERA_USART_RCC, ENABLE);

	/* PB10：USART3_TX。 */
	GPIO_InitStructure.GPIO_Pin = CAMERA_USART_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(CAMERA_USART_GPIO, &GPIO_InitStructure);

	/* PB11：USART3_RX。 */
	GPIO_InitStructure.GPIO_Pin = CAMERA_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(CAMERA_USART_GPIO, &GPIO_InitStructure);

	/* 串口格式：8 位数据位、无校验、1 位停止位、无硬件流控。 */
	USART_InitStructure.USART_BaudRate = CAMERA_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(CAMERA_USART, &USART_InitStructure);

	/* 使能 USART3 接收中断。 */
	NVIC_InitStructure.NVIC_IRQChannel = CAMERA_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	CAMERA_USART->SR; CAMERA_USART->DR;
	USART_ClearITPendingBit(CAMERA_USART, USART_IT_RXNE);
	USART_ITConfig(CAMERA_USART, USART_IT_RXNE, ENABLE);
	USART_Cmd(CAMERA_USART, ENABLE);
}

/**
 * @brief  摄像头 USART3 接收中断函数。
 * @note   解析固定 5 字节协议帧：0xAA + mode + dx + dy + 0x55。
 */
void USART3_IRQHandler(void)
{
	uint8_t data;

	if(USART_GetITStatus(CAMERA_USART, USART_IT_RXNE) != RESET)
	{
		data = (uint8_t)CAMERA_USART->DR;

		/* 未进入一帧时，先等待帧头。 */
		if(g_cameraRxCount == 0)
		{
			if(data == CAMERA_PROTOCOL_HEADER)
			{
				g_cameraRxBuf[g_cameraRxCount++] = data;
			}
		}
		else
		{
			/* 收集固定长度 5 字节协议帧。 */
			g_cameraRxBuf[g_cameraRxCount++] = data;

			if(g_cameraRxCount >= 5)
			{
				if(g_cameraRxBuf[0] == CAMERA_PROTOCOL_HEADER && data == CAMERA_PROTOCOL_TAIL)
				{
					/* 摄像头发送的 dx/dy 是 int8 有符号偏差值。 */
					g_cameraFrame.mode = g_cameraRxBuf[1];
					g_cameraFrame.dx = (int8_t)g_cameraRxBuf[2];
					g_cameraFrame.dy = (int8_t)g_cameraRxBuf[3];
					g_cameraFrameReady = 1;
				}

				/* 如果当前字节正好也是帧头，则作为下一帧的起点。 */
				g_cameraRxCount = (data == CAMERA_PROTOCOL_HEADER) ? 1 : 0;
				if(g_cameraRxCount == 1)
				{
					g_cameraRxBuf[0] = data;
				}
			}
		}

		USART_ClearITPendingBit(CAMERA_USART, USART_IT_RXNE);
	}
}

/**
 * @brief  读取最新一帧完整的摄像头数据。
 * @param  frame 输出参数，用于返回 mode/dx/dy。
 * @return 1 表示读到新帧，0 表示暂无新帧。
 */
uint8_t Camera_GetFrame(CameraFrame_t *frame)
{
	uint8_t hasFrame = 0;

	if(frame == 0)
	{
		return 0;
	}

	/* 与 USART3_IRQHandler 共享数据，复制时短暂关中断保护。 */
	__disable_irq();
	if(g_cameraFrameReady)
	{
		frame->mode = g_cameraFrame.mode;
		frame->dx = g_cameraFrame.dx;
		frame->dy = g_cameraFrame.dy;
		g_cameraFrameReady = 0;
		hasFrame = 1;
	}
	__enable_irq();

	return hasFrame;
}

/**
 * @brief  清空已缓存的摄像头数据，并重新开始协议解析。
 */
void Camera_ResetFrameState(void)
{
	__disable_irq();
	g_cameraFrame.mode = 0;
	g_cameraFrame.dx = 0;
	g_cameraFrame.dy = 0;
	g_cameraFrameReady = 0;
	g_cameraRxCount = 0;
	__enable_irq();
}
