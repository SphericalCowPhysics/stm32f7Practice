
#include "hwConfig.h"
#include "regs.h"
#include "commands.h"
#include "usbd_cdc_if.h"

int VCP_read(void *pBuffer, int size);
int UsbVcp_write(const void *pBuffer, int size);
void BlinkLed();

uint16_t FirmWareVersion = 1;
//version 1, VID/PID = 0x0483/0x5740, defined in usbd_desc.c

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	InitUsb();
	MX_SPI2_Init();
	MX_ADC1_Init();
	InitRegs();

	char byte;
	while (1)
	{
		//BlinkLed(); //cannot do this while using SPI because of pin conflict
		if (rxLen > 0) {
			Parse();
		}
	}
}

uint32_t lastBlink = 0;
void BlinkLed()
{
	static uint32_t blinkTime = 500; //on/off time
	uint32_t sysTick = HAL_GetTick();
  
	if ((sysTick - lastBlink) > blinkTime) {
		HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_1);
		lastBlink = sysTick;
	}
}
