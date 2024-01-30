
#include "hwConfig.h"
#include "regs.h"
#include "commands.h"
#include "usbd_cdc_if.h"
#include "main.h"

int VCP_read(void *pBuffer, int size);
int UsbVcp_write(const void *pBuffer, int size);
//void BitBangMotorStepping(uint16_t steps, uint16_t duration, uint8_t direction);
void BitBangMoveMotor(uint16_t steps, uint16_t timePerStep, uint16_t direction);
void CheckMotorUpdate();
void BitBangCheckEncoderUpdate();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint32_t motorPulseStartTime;
uint16_t FirmWareVersion = 1;
//version 1, VID/PID = 0x0483/0x5740, defined in usbd_desc.c
uint32_t r;
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	InitUsb();
	MX_SPI2_Init();
	MX_ADC1_Init();
	//MX_TIM2_Init();
	//MX_TIM3_Init();
	InitRegs();

	/*Initialize various variables used for testing*/
	char byte;
	int c,d = 2;
	uint32_t sysTick;
	uint32_t lastTick = 0;
	uint32_t durationTick;
	uint32_t e = 0;
	uint32_t cw, ccw = 0;
	int totalRot = 0;
	//BitBangMoveMotor(200, 2, 0);
	//for (d = 1; d <= 100; d++) {for (c = 1; c <= 32767; c++) {}}  //Wastes time during initialization to allow breakpoints on first pass inside while loop
	while (1)
	{			
		 //For testing motor movements inside fimrware
		 /*e++;;
		if (e == 2)
		{
			if (c % 2 == 0)
			{
				BitBangMoveMotor(1, 2, 0);
				//CheckMotorUpdate();	
				c++;
			}
			else
			{
				BitBangMoveMotor(1, 2, 1);
				//CheckMotorUpdate();	
				c++;
			}
			e = 0;
		}*/
		
		//For testing the Encoder interrupts.
		/*
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, 0);
		if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7) == 0)
		{
			e++;
		}
		if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7) == 1)
		{
			e++;
		}
		*/
		cw = Regs.u16[RegEncoderCwSteps];
		ccw = Regs.u16[RegEncoderCCwSteps];
		totalRot = cw - ccw;		
		CheckMotorUpdate();	
		//BitBangCheckEncoderUpdate();
		if (rxLen > 0) {
			Parse();
		}

	}
}

#ifdef BitBangMotor
void BitBangMoveMotor(uint16_t steps, uint16_t timePerStep, uint16_t direction)
{
	Regs.u16[RegMotorSteps] = steps;
	Regs.u16[RegMotorStepTime] = timePerStep;
	Regs.u16[RegMotorDirEnable] = (0x8000 | direction);
}

void CheckMotorUpdate()
{
	static uint32_t numStepsCompleted = 0, timeAtLastStepChange = 0, stepPolarity = 0;
	uint16_t stepTime;		//Step time should be divisible by 2 to avoide timing errors, or by 4 if stepTime is 2mS or 4mS
	if (Regs.u16[RegMotorStepTime] <= 4)
		{stepTime = Regs.u16[RegMotorStepTime] / 2;}	//Timing error occurs due to 'resolution' of sysTick. Overcompensate by making stepTime even smaller
	else
		{stepTime = Regs.u16[RegMotorStepTime];}
	
	if ((Regs.u16[RegMotorDirEnable] & 0x8000) == 0x8000)
	{	
		motorPulseStartTime = HAL_GetTick();		//Currently initialized in SetReg() to avoid first pulse delay problem but it didn't help
		//is it is time now to change step?
		if ( ((motorPulseStartTime - timeAtLastStepChange) > (stepTime / 2)) )		//satisfied if half a period has elapsed.
		{
			if (numStepsCompleted <= Regs.u16[RegMotorSteps])							//satisfied if we still have to do more steps. Probably unnecessary  given that satisfying last if makes first if FALSE
			{
				if ((Regs.u16[RegMotorDirEnable] & 1) == 1)
					{HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);}		//set direction of rotation to CW looking toward load
				else
					{HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);}		//set direction of rotation to CCW looking toward load
			
				if (0 == stepPolarity)
				{
					HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);			//Take 1 step
					numStepsCompleted++;
					stepPolarity = 1;
				}
				else
				{
					HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);		//Reset to get ready to take another step
					stepPolarity = 0;
				}
			}
			
			timeAtLastStepChange = motorPulseStartTime;										//update the time checker for 'is it time to change step if statement
		}
		//if we have done all the steps, then reset the motor move flag, and stop toggling step
		if ((numStepsCompleted == Regs.u16[RegMotorSteps]) && (stepPolarity == 0))	//satisfied if we do all the steps AND completed the final cycle and return to active LOW
		{
			//tell motor to stop stepping, and reset numStepCompleted to prepare for next motor moving command
			Regs.u16[RegMotorDirEnable] &= ~0x8000;			//equivalent to Reg = ( Reg & (01...1) ). Essentially always sets MSB (enable) to 0, while LSB (direction) keeps its current value.
			Regs.u16[RegMotorSteps], numStepsCompleted = 0;	//reset these values in preperation for another USB command to move motor
			motorPulseStartTime = 0;
		}
	}

}

void BitBangCheckEncoderUpdate()
{
	/*If Encoder Out A rises while B is LOW (A rises first), then add 1 to clockwise counter. If Out A rises while B is HIGH (B rises first), then add 1 to counter clockwise counter
	 *Above can only be implemented using interrupts instead implement the following:
	 *Keep track of the last known value of Out A. Using sysTick, check every tick if the value of Out A has changed, if so, ... THIS WILL NOT WORK: sysTick does not have necessary resolution 
	 *Use interrupts instead
	 */
}
	
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//If Encoder Out A (PG6) is rising or falling, then check against Encoder Out B (PG7). If They are not equal, that means that A rises first, so a clockwise step (dir=1) was just completed.
	/*if (GPIO_Pin == GPIO_PIN_6)
	{
		//Set PG6 as rising/falling trigger, and PG7 as input. The conditional statement should work according to scope. 
		//The encoder pulses are only around 4kHz, so the interrupt should be fast enough, but the if-condition is never satisfied. Seems related to inability to trigger on falling edge, or to ever read PG6 or PG7 == 0
		if ( (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6) != HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7)) )
		{
			Regs.u16[RegEncoderCwSteps]++;
		}
		else
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}
	}*/
	
	
	/* //Use heartbeat LED to see when interrupt is being triggered
	if (GPIO_Pin == GPIO_PIN_6)
	{
		if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7) == GPIO_PIN_RESET)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		}
		if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7) == GPIO_PIN_SET)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		}
	}*/	
	
	
	//Try both PG6 and 7 as a rising edge interrupt
	
	if (GPIO_Pin == GPIO_PIN_7)
	{
		if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6) == GPIO_PIN_RESET)		//Could maybe replace if statment with ccw += ReadPin6 and cw +=  ~ReadPin6
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}
		else
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}	
	}
	else if (GPIO_Pin == GPIO_PIN_6)
	{
		if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7) == GPIO_PIN_RESET)		//Could maybe replace if statment with ccw += ReadPin7 and cw +=  ~ReadPin7
		{
			Regs.u16[RegEncoderCwSteps]++;
		}
		else
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}
	}
#endif	
	


#ifdef ObsoleteBitBangMotor
	void BitBangMotorStepping(uint16_t steps, uint16_t duration, uint8_t direction)
	{
		SetReg(RegMotorSteps, steps);
		//uint16_t stepsRemaining = ReadReg(RegMotorSteps);
		uint16_t stepsRemaining = steps;
		SetReg(RegMotorStepsDuration, duration);
		//uint16_t ticksRemaining = ReadReg(RegMotorStepsDuration);
		uint16_t stepInterval = (int)(duration / steps);					//How many mS to wait between each step
		SetReg(RegMotorDirEnable, (0x80 | direction));						//OR sets MSB to 1 and LSB to 0 or 1 depending on value of direction. Enables motor movement, and sets DIR to CW if direction==1
		/*****Get REGS working
		if ((ReadReg(RegMotorDirEnable) & 1) == 1) 
		{
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);				//set direction of rotation. Is this CW or CCW	
		}
		else
		{
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);			//set direction of rotation. Is this CW or CCW
		}
		if ((ReadReg(RegMotorDirEnable) & 0x80) == 0x80) 
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);				//enables motor motion	
		}
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);				//set direction of rotation to 1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);					//enables motor motion
		*/

		uint32_t sysTick;
		uint32_t lastTick = 0;
		int waste;
		while (stepsRemaining > 0)
		{
			sysTick = HAL_GetTick();
			if (sysTick % stepInterval == 0 && sysTick > lastTick) 
			{
				/*
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);			//Take 1 step
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);		//Prepare to take another step
				*/
				SetReg(RegMotorSteps, stepsRemaining);
				stepsRemaining--;
				lastTick = sysTick;
				//for (waste = 1; waste <= 5000; waste++)
				//{}
			}
		
		}
		r = ReadReg(RegMotorDirEnable);
		SetReg(RegMotorDirEnable, (0x80 ^ r));		//XOR sets shared MSB to 0, leaves all other bits alone. Essentially sets MSB to 0
		/*Get REGS working
		if (ReadReg(RegMotorDirEnable) && 0x80 != 0x80) 
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);			//disables motor motion
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);			//disables motor motion
		*/
		for (waste = 1; waste < 30000; waste++)
		{}
	
	
	}
#endif		
	
#ifdef InterruptTesting
	if (GPIO_Pin == GPIO_PIN_11)
	{
		HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_1);
	}
#endif
}