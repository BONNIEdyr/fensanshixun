#include "board.h"
#include "config.h"
#include "camera.h"

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

/**
	* @brief   ����NVIC������
	* @param   ��
	* @retval  ��
	*/
void nvic_init(void)
{	
	// 4bit��ռ���ȼ�λ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannel = MOTOR_USART_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

/**
	*	@brief		����ʱ�ӳ�ʼ��
	*	@param		��
	*	@retval		��
	*/
void clock_init(void)
{
	// ʹ��GPIOA��AFIO����ʱ��
	RCC_APB2PeriphClockCmd(MOTOR_USART_GPIO_RCC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// ʹ��USART1����ʱ��
	RCC_APB2PeriphClockCmd(MOTOR_USART_RCC, ENABLE);

	// ����JTAG
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

/**
	* @brief   ��ʼ��USART
	* @param   ��
	* @retval  ��
	*/
void usart_init(void)
{
/**********************************************************
***	��ʼ��USART1����
**********************************************************/
	// PA9 - USART1_TX
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = MOTOR_USART_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* ����������� */
	GPIO_Init(MOTOR_USART_GPIO, &GPIO_InitStructure);
	// PA10 - USART1_RX
	GPIO_InitStructure.GPIO_Pin = MOTOR_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					/* �������� */
	GPIO_Init(MOTOR_USART_GPIO, &GPIO_InitStructure);

/**********************************************************
***	��ʼ��USART1
**********************************************************/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = MOTOR_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(MOTOR_USART, &USART_InitStructure);

/**********************************************************
***	���USART1�ж�
**********************************************************/
	MOTOR_USART->SR; MOTOR_USART->DR;
	USART_ClearITPendingBit(MOTOR_USART, USART_IT_RXNE);

/**********************************************************
***	ʹ��USART1�ж�
**********************************************************/	
	USART_ITConfig(MOTOR_USART, USART_IT_RXNE, ENABLE);
	USART_ITConfig(MOTOR_USART, USART_IT_IDLE, ENABLE);

/**********************************************************
***	ʹ��USART1
**********************************************************/
	USART_Cmd(MOTOR_USART, ENABLE);
}

/**
	*	@brief		���س�ʼ��
	*	@param		��
	*	@retval		��
	*/
void board_init(void)
{
	nvic_init();
	clock_init();
	usart_init();
	Camera_Init();
}
