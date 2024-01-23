
#include "hwConfig.h"
#include "regs.h"
#include "commands.h"
#include "usbd_cdc_if.h"
//#include "regs.c"
#include "main.h"

int VCP_read(void *pBuffer, int size);
int UsbVcp_write(const void *pBuffer, int size);
void BlinkLed();

uint16_t FirmWareVersion = 1;
//version 1, VID/PID = 0x0483/0x5740, defined in usbd_desc.c

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	//HAL_SetTickFreq(HAL_TICK_FREQ_10kHZ);		//10kHz => 100uS SysTicks => 83 samples/120HzPulse(8.3mS)
	MX_GPIO_Init();
	InitUsb();
	//MX_SPI2_Init();
	MX_TIM10_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	//MX_TIM1_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	//MPU_Config();
	InitRegs();		

	char byte;
	
	while (1)
	{
		
		//uint16_t dummy1 = ReadReg(RegAdc3TemperatureSense);
		uint16_t dummy2 = ReadReg(RegAdc3PhotoCurrent);
		/*
		 *uint16_t dummy3 = ReadReg(RegAdcTemp);
		uint16_t dummy4 = ReadReg(RegAdcRef);
		uint16_t dummy5 = ReadReg(RegTick);
		uint16_t dummy6 = ReadReg(RegUniqueID);
		uint16_t dummy7 = ReadReg(RegFirmWareVersion);
		*/
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
