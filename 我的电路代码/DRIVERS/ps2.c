#include "ps2.h"
#include "config.h"

#define DI_READ()       GPIO_ReadInputDataBit(PS2_GPIO, PS2_DI_PIN)
#define CMD_HIGH()      GPIO_SetBits(PS2_GPIO, PS2_CMD_PIN)
#define CMD_LOW()       GPIO_ResetBits(PS2_GPIO, PS2_CMD_PIN)
#define CS_HIGH()       GPIO_SetBits(PS2_GPIO, PS2_CS_PIN)
#define CS_LOW()        GPIO_ResetBits(PS2_GPIO, PS2_CS_PIN)
#define CLK_HIGH()      GPIO_SetBits(PS2_GPIO, PS2_CLK_PIN)
#define CLK_LOW()       GPIO_ResetBits(PS2_GPIO, PS2_CLK_PIN)

static const uint8_t ps2_cmd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t ps2_data[9] = {0};

static void PS2_DelayUs(uint32_t us)
{
	__IO uint32_t i;
	while(us--)
	{
		for(i = 0; i < (SystemCoreClock / 8000000U); i++)
		{
		}
	}
}

void PS2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(PS2_RCC_GPIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = PS2_DI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PS2_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PS2_CMD_PIN | PS2_CS_PIN | PS2_CLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PS2_GPIO, &GPIO_InitStructure);

	CMD_HIGH();
	CLK_HIGH();
	CS_HIGH();
}

static uint8_t PS2_ReadWriteData(uint8_t cmd)
{
	uint8_t res = 0;
	uint8_t ref;

	for(ref = 0x01; ref > 0x00; ref <<= 1)
	{
		if(ref & cmd)
		{
			CMD_HIGH();
		}
		else
		{
			CMD_LOW();
		}

		CLK_LOW();
		PS2_DelayUs(PS2_BIT_DELAY_US);

		if(DI_READ())
		{
			res |= ref;
		}

		CLK_HIGH();
		PS2_DelayUs(PS2_BIT_DELAY_US);
	}

	return res;
}

void PS2_ScanKey(PS2_JoystickTypeDef *joystick)
{
	uint8_t i;

	CS_LOW();
	for(i = 0; i < 9; i++)
	{
		ps2_data[i] = PS2_ReadWriteData(ps2_cmd[i]);
	}
	CS_HIGH();

	joystick->mode = ps2_data[1];
	joystick->btn1 = (uint8_t)(~ps2_data[3]);
	joystick->btn2 = (uint8_t)(~ps2_data[4]);
	joystick->RJoy_LR = ps2_data[5];
	joystick->RJoy_UD = ps2_data[6];
	joystick->LJoy_LR = ps2_data[7];
	joystick->LJoy_UD = ps2_data[8];
}
