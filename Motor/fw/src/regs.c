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
void ReadLoadCell();
//uint8_t directionEnable = 1;
//uint16_t waste;

void InitRegs() {
	//Power-on intitializations
	Regs.u16[RegMotorStepTime] = 2;
	Regs.u16[RegMotorSteps], Regs.u16[RegMotorDirEnable] = 0;
	Regs.u16[RegEncoderCwSteps], Regs.u16[RegEncoderCCwSteps] = 0;						//Either use Cw/CCwSteps if using GPIO interrupts, or use Position/Velocity if using Timer Encoder interrupts
	Regs.u16[RegEncoderPosition], Regs.u16[RegEncoderVelocity] = 0;
	Regs.u16[RegLoadCellUpper16], Regs.u16[RegLoadCellLower8], Regs.u16[RegLoadCellOffset] = 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);				//electro-mechanically enables motor motion
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
		case RegLoadCellUpper16:
			ReadLoadCell();
			break;
		case RegLoadCellLower8:
			//Do nothing. Reading Upper16 runs ReadLoadCell() which fills all 24 bits of data. Lower8 only exists because my Regs only support 16 bit data types
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
    
    switch (nReg) 
    {
		case RegMotorDirEnable:
	    Regs.u16[nReg] = value;
	    /*Tried to fix first pulse delay problem
	    if ( (value & 0x8000) == 0x8000)
	    {
		    motorPulseStartTime = HAL_GetTick();
		}
		*/
			break;
		case RegMotorSteps:
			Regs.u16[nReg] = value;
			break;
		case RegMotorStepTime:
			Regs.u16[nReg] = value;
			break;	
    case RegLoadCellOffset:
	    Regs.u16[nReg] = value;
	    break;	
	    /*	//I shouldn't really ever need to reset these
		case RegEncoderCwSteps:
			Regs.u16[nReg] = value;
			break;
		case RegEncoderCCwSteps:
			Regs.u16[nReg] = value;
			break;
	    */
	    /*	//DO NOT CALL SETREG TO DO THINGS. SetReg should be reserved only for commands received over USB via Parse()
		case RegMotorDirEnable:
			Regs.u8[nReg] = value;
		    directionEnable = value;
			if ((value & 1) == 1)
			{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);				//set direction of rotation to CW looking toward load
			}
			else
			{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);			//set direction of rotation to CCW looking toward load
			}
			if ((value & 0x80) == 0x80)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);				//enables motor motion
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);			//disables motor motion
			}  
			break;
		case RegMotorSteps:
			Regs.u16[nReg] = value;
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);					//Take 1 step
			for (waste = 1; waste < 0x1D00; waste++)
			{}
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);				//Prepare to take another step
			break;
		case RegMotorStepTime:
			Regs.u16[nReg] = value;
			break;
	    */
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
	//Need to use PA8 as timer1 not chip select
	//if (0 == level)
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	
	//else
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	
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

void ReadLoadCell()
{
	//Let the PWM (PF9) send 24+gain pulses. During the first 24 pulses, DOut (PF8) will will send a bit of data from MSB to LSB. The last pulse(s) sets gain and channel
	//PWM should have a 50% duty and a period of 0.4-100us. Without any delaying commands it is 1.5us.
	//At the begining of a new transmission, the PWM must wait at least 0.1us after DOut goes low (start transmission) before it starts sending pulses.

	uint16_t data[2] = { 0 };	
		

	if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8) == 0)		//DOut must start low before first clock rising edge to signal that transmissions are allowed
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);		//Clock starts low
		//Get 16 upper bits
		for (int i = 0; i < 16; i++)						
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 1);						//Clock goes high to receive a bit of data
			data[0] |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8) << (15 - i);		//Data is received and stored starting with MSB
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);						//Clock goes low again	
		}	

		//Get 8 lower bits
		for (int i = 0; i < 8; i++)			
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 1);						//Clock goes high to receive data
			data[1] |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8) << (15 - i);		//Data is received and stored. The last 8 bits will remain 0, and should ultimatley be right shifted out when final data is read
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);						//Clock goes low again	
		}
		//Pulse the clock an extra time once, twice or thrice depending on desired gain and channel. By default, use 1 extra pulse for 128 gain on ChA
		for (int i = 0; i < LoadCell_ChA_Gain128; i++)		
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 1);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);
		}
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 0);	//Ensure clock is low to prepare for next read command

		Regs.u16[RegLoadCellUpper16] = data[0];
		Regs.u16[RegLoadCellLower8] = data[1];
	}
}



