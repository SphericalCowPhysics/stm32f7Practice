/**
  ******************************************************************************
  * @file           : regs.c
  * @brief          : Code to handle registers that do not survive power cycle
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "regs.h"
#include "hwConfig.h"
#include "usbd_cdc_if.h"
#include "math.h"

volatile REG_BLOCK Regs;
extern uint16_t FirmWareVersion;
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;

uint16_t ReadAdc(uint16_t chan);
void SetCSN(uint8_t level);
void WriteSpi8(uint8_t reg, uint8_t val);
void WriteSpi16(uint8_t reg, uint16_t val);
uint8_t ReadSpi8(uint8_t reg);
int16_t ReadSpi16(uint8_t reg);
void InitImu(void);

void InitRegs() {
	//Power-on intitializations
	InitImu();
}	

//UpdateRegs called periodically in main()
uint8_t UpdateRegs() {
    uint8_t err = 0;
    static uint16_t nReg=0;
    
    err = ReadReg(nReg);
    nReg++;
    if (nReg>=RegLast || nReg>=REG_SIZE16) nReg = 0;
    
    return err;
}

uint8_t ReadReg(uint16_t nReg) {
	//This routine called if a read register command received over USB, or periodically to update volatile registers.
	//In most cases registers are always valid, but for some the value needs to be updated.
	uint8_t err = 0;
	uint16_t temp = 0;
    
    if (nReg>=RegLast || nReg>=REG_SIZE16) return 0; //not an error -- just don't read past the end of the register block
    
    switch (nReg) {
        case RegFirmWareVersion:
	        Regs.u16[nReg] = FirmWareVersion;
            break;
		case RegUniqueID:
			temp = *(uint16_t *)(0x1FF0F420UL);
//			temp ^= *(uint16_t *)(0x1FF0F422UL);
//			temp ^= *(uint16_t *)(0x1FF0F424UL);
//			temp ^= *(uint16_t *)(0x1FF0F426UL);
			Regs.u16[nReg] = temp;
			break;
		case RegTick:
			Regs.u16[nReg] = (uint16_t)(HAL_GetTick() & 0xFFFF);
			break;
		case RegAdcTemp:
			Regs.u16[nReg] = ReadAdc(16); //temperature sensor is channel 16, 0.76V @25degC + 2.5mV/degC, temp = 25+(nAdc*3.3/4095-0.76)/0.025, 1022 => 27.5degC
			break;
		case RegAdcRef:
			Regs.u16[nReg] = ReadAdc(17); //internal reference voltage (1.2Vnom), should be around 1.2/3.3*4095 = 1500, can use to calculate voltage of 3.3V supply
			break;
		case RegImuWhoAmI: //should read 0x71 = 113
			Regs.u16[nReg] = ReadSpi8(0x75);
			break;
		case RegImuAx:
			Regs.s16[nReg] = ReadSpi16(0x3b);
			break;
		case RegImuAy:
			Regs.s16[nReg] = ReadSpi16(0x3d);
			break;
		case RegImuAz:
			Regs.s16[nReg] = ReadSpi16(0x3f);
			break;
		default:
			break;
    }  
    return err;
}

void SetReg(uint16_t nReg, uint16_t value) 
{   
    int16_t sval;
	//This routine called if a write register command received over USB
    if (nReg>=RegLast || nReg>=REG_SIZE16) return;
    
    switch (nReg) {
//		case RegAdcTemp:
//			Regs.u16[nReg] = value;
//			//SetChannel(value);
//			break;
		default:
			break;
    }    
    ReadReg(nReg);
}

uint16_t ReadAdc(uint16_t chan)
{
	ADC_ChannelConfTypeDef sConfig = { 0 };
	if (16 == chan)
		sConfig.Channel = ADC_CHANNEL_16;
	else if (17 == chan)
		sConfig.Channel = ADC_CHANNEL_17;
	else if (0 == chan)
		sConfig.Channel = ADC_CHANNEL_0;
	else
		sConfig.Channel = chan;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	uint16_t retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (retVal);
}


void SetCSN(uint8_t level)
{
	if (0 == level)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	
}

//write a byte 
void WriteSpi8(uint8_t reg, uint8_t val)
{
	uint8_t controlwords[2];
	controlwords[0] = reg; //must be less than 0x7f or it will be interpreted as a read
	controlwords[1] = val;    //value to write to register
	SetCSN(0);
	HAL_SPI_Transmit(&hspi2, controlwords, 2, 1000);
	SetCSN(1);
}

//write a word
void WriteSpi16(uint8_t reg, uint16_t val)
{
	uint8_t controlwords[3];
	controlwords[0] = reg; //must be less than 0x7f or it will be interpreted as a read
	controlwords[1] = (val >> 8)&0xff;    //value to write to register
	controlwords[2] = val&0xff;    //value to write to register
	SetCSN(0);
	HAL_SPI_Transmit(&hspi2, controlwords, 3, 1000);
	SetCSN(1);
}

//read a byte 
uint8_t ReadSpi8(uint8_t reg)
{
	uint8_t controlwords[2];
	controlwords[0] = 0x80 | reg;
	uint8_t rcvDat[2];
	SetCSN(0);
	HAL_SPI_TransmitReceive(&hspi2, controlwords, rcvDat, 2, 1000);
	SetCSN(1);
	return (rcvDat[1]);
}

//read a signed short 
int16_t ReadSpi16(uint8_t reg)
{
	uint8_t controlwords[3];
	controlwords[0] = 0x80 | reg;
	uint8_t rcvDat[3];
	SetCSN(0);
	HAL_SPI_TransmitReceive(&hspi2, controlwords, rcvDat, 3, 1000);
	SetCSN(1);
	return ((rcvDat[1] << 8) | rcvDat[2]);
}

void InitImu(void)
{
	WriteSpi8(0x6b, 0x01); //PWR_MGMT_1 register, select best available clock
	WriteSpi8(0x1a, 0x06); //CONFIG register, 5Hz filter for gyro
	WriteSpi8(0x1b, 0x00); //GYRO_CONFIG register, 250dps full scale
	WriteSpi8(0x1c, 0x00); //ACCEL_CONFIG_1 register, 2g full scale
	WriteSpi8(0x1d, 0x06); //ACCEL_CONFIG_2 register, 5Hz filter
}
