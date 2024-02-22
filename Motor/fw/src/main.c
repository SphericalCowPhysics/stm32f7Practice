
#include "hwConfig.h"
#include "regs.h"
#include "commands.h"
#include "usbd_cdc_if.h"
#include "main.h"

int VCP_read(void *pBuffer, int size);
int UsbVcp_write(const void *pBuffer, int size);
void BitBangMoveMotor(uint16_t steps, uint16_t timePerStep, uint16_t direction);
void CheckMotorUpdateBb();
void CheckMotorUpdatePwm();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void UpdateEncoder(TIM_HandleTypeDef *htim);					//Used code found at https://www.steppeschool.com/pages/blog/stm32-timer-encoder-mode. Can't test functionality because encoder interrupt not triggering
void AdjustPWM(TIM_HandleTypeDef* htim, uint32_t Channel, uint16_t period, uint16_t pulseWidth);
uint32_t motorPulseStartTime;									//used by bitbanged motor moving to set up the bit banged timer
uint16_t FirmWareVersion = 1;
uint32_t numMotorStepsCompleted;								//used to stop motor when enough steps have been completed
int motorIsMoving = 0;					
//version 1, VID/PID = 0x0483/0x5740, defined in usbd_desc.c
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	InitUsb();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	//MX_SPI2_Init();
	//MX_ADC1_Init();
	//MX_TIM2_Init();
	//MX_TIM3_Init();
	InitRegs();
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);			// main channel
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);			// enslaved channel
	//AdjustPWM(&htim1, TIM_CHANNEL_1, 8000, 4000);		//Motor PWM should be adjusted and started by writing to reg/ CheckMotorUpdatePwm()
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	TIM3->CNT = 0x8000-0x1000;
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	
	/*Initialize various variables used for testing*/

	//for (d = 1; d <= 100; d++) {for (c = 1; c <= 32767; c++) {}}  //Wastes time during initialization to allow breakpoints on first pass inside while loop

	while (1)
	{			
		 //For testing motor movements inside fimrware
		 /*e++;
		if (e == 10000)
		{
			if (c % 2 == 0)
			{
				BitBangMoveMotor(10, 2, 0);
				//CheckMotorUpdate();	
				c++;
			}
			else
			{
				BitBangMoveMotor(20, 2, 1);
				//CheckMotorUpdate();	
				c++;
			}
			e = 0;
		}
		*/
		
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
		/* Weird test to see if pins are actually rising and falling
		if ( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) == GPIO_PIN_SET) && (qpol == 0) )
		{
			q1++;
			qpol = 1;
		}
		else if ( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) == GPIO_PIN_RESET)  && (qpol == 1) )
		{
			q2++;
			qpol = 0;
		}
		if ( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7) == GPIO_PIN_SET) && (qqpol == 0) )
		{
			qq1++;
			qqpol = 1;
		}
		else if ( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7) == GPIO_PIN_RESET)   && (qqpol == 1) )
		{
			qq2++;
			qqpol = 0;
		}
		*/
		/*
		cw = Regs.u16[RegEncoderCwSteps];
		ccw = Regs.u16[RegEncoderCCwSteps];
		totalRot = cw - ccw;
		*/
	
		//For testing adjustments to PWM during runtime
		/*//TIM1->ARR = 4000;
		AdjustPWM(&htim1, TIM_CHANNEL_1, 8000, 1000);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		//for (int ij = 0; ij < 100000-1; ij++) {}
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		for (int ij = 0; ij < 100000-1; ij++) {}
		
		AdjustPWM(&htim1, TIM_CHANNEL_1, 8000, 7000);

		//TIM1->ARR = 8000;
		//TIM2->CCR1 = 7000;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		//for (int ij = 0; ij < 100000-1; ij++) {}
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		for (int ij = 0; ij < 100000-1; ij++) {}
		*/		

		CheckMotorUpdatePwm();	
		if (rxLen > 0) {
			Parse();
		}

	}
}
/*This function can be used to send move commands to the motor from inside firmware. It is prefered to send such commands over USB from GUI or python via parse.*/
void BitBangMoveMotor(uint16_t steps, uint16_t timePerStep, uint16_t direction)
{
	Regs.u16[RegMotorSteps] = steps;
	Regs.u16[RegMotorStepTime] = timePerStep;
	Regs.u16[RegMotorDirEnable] = (0x8000 | direction);
}
/*
 *A computationally intensive way to move the motor, and step time is wildly inaccurate because sysTick is comparable to the prefered 2ms step time of stepper motor.
 *The first if/else is a sloppy way of trying to correct for the sysTick resolution problem.
 *The nested if statements essentially check if the Enable bit (MSB of RegMotorDirEnable) was set to 1 which must always accompany a USB command to move the motor, then checks if more steps must be completed.
 *If so, the GPIO controlling motor direction is set/reset based on the value of RegMotorSteps which typically accompany a USB command to move the motor.
 *Then a step is either taken (rising edge of a bitbanged timer) and numStepsCompleted is incremented, or the bitbanged timer resets in preperation for the next step
 *Lastly, if all steps have been completed, reset all relevent values: most importantly the Enable bit is set to 0, preventing anything in the nested if statements from running until a new USB command is received.
*/
#ifdef BitBangMotor

void CheckMotorUpdateBb()
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
				if ( (Regs.u16[RegMotorDirEnable] & 1) == 1)
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
#endif
/*
 *The prefered way to move the motor. The motor is driven by a PWM on Tim1Ch1. When Enable bit is 1, and the motor is not already moving, PWM is initialized and started.
 *PWM period (ie TIM1->ARR) is by default 8000 events. APB2 Timer runs at 4MHz, so there are 8000 events every 2 milisecond. 
 *Fastest step allowed is 0.00025ms (4Mhz means an event every 250ns) at ARR=1; and the slowest allowed is about 16ms (65,535 * 2/8000) at ARR=65535.
 *In hindsight, probably should have run clock faster and just adjusted the prescaler. For some reason motor rattles awefully if RegMotorStepTime is less than 1.
*/
void CheckMotorUpdatePwm()
{
	static uint16_t motorPeriod;
	//I would prefer these next two lines go inside first if statment, but it may have been causing issues. Changed some things so it may now be OK to return them inside if statement.
	motorPeriod = (uint16_t)((Regs.u16[RegMotorStepTime] * MotorPWM_EventsPer2ms) - 1);		//APB2 Timer runs at 4Mhz, so there are 8000 events every 2 milisecond	
	AdjustPWM(&htim1, TIM_CHANNEL_1, motorPeriod, (uint32_t)(motorPeriod / 2));				//This division or assignment for the pulse width as a funciton of motorPeriod was causing problems. Hardcode 1000 as default if required
	if ( (Regs.u16[RegMotorDirEnable] & 0x8000) == 0x8000 && motorIsMoving == 0)			//MSB of DirEnable set to 1 when a move command is received over usb and 0 when that move command completes. motorIsMoving simply prevents AdjustPwm() at TimStart from being repeatedly called when motor is already moving
	{

		if ( (Regs.u16[RegMotorDirEnable] & 1) == 1) {
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);			//set direction of rotation to CW looking toward load
		}		
		else {
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);		//set direction of rotation to CCW looking toward load
		}		
		//AdjustPWM(&htim1, TIM_CHANNEL_1, motorPeriod, 1000);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		motorIsMoving = 1;
	}
	
	else if(numMotorStepsCompleted >= Regs.u16[RegMotorSteps] && motorIsMoving == 1)		//This could be moved into HAL_TIM_IC_CaptureCallback to guarantee to extra steps are ever taken. 
	{
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		/*Reset all relevent values to prepare for another move order*/
		TIM1->CNT = 0;									//Clear any left over counts so first pulse of next move command doesn't occur too quickly
		Regs.u16[RegMotorDirEnable] &= ~0x8000;			//equivalent to Reg = ( Reg & (011...11) ). Essentially always sets MSB (enable) to 0, while LSB (direction) keeps its current value.
		numMotorStepsCompleted = 0;
		motorIsMoving = 0;								//Allow PWM to be adjusted and started if another move command is received (ie MSB of DirEnable reg == 1)
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_6)
	{	
		//If Encoder Out A (PG6) is rising or falling, then check against Encoder Out B (PG7). If They are not equal, that means that A rises first, so a clockwise step (dir=1) was just completed.
		if ((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) != HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7))) {
			Regs.u16[RegEncoderCwSteps]++;
		}
		else {
			Regs.u16[RegEncoderCCwSteps]++;
		}
	}
	
	if (GPIO_Pin == GPIO_PIN_7)
	{	
		//If Encoder Out B (PG7) is rising or falling, then check against Encoder Out A (PG6). If They ARE equal, that means that B rises second, so a clockwise step (dir=1) was just completed.
		if ((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7) == HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6))) {
			Regs.u16[RegEncoderCwSteps]++;
		}
		else {
			Regs.u16[RegEncoderCCwSteps]++;
		}
	}
	/*//Try both PG6 and 7 as a rising edge interrupt
	if (GPIO_Pin == GPIO_PIN_6)
	{
		if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7) == GPIO_PIN_RESET)		//Could maybe replace if statment with ccw += ReadPin6 and cw +=  ~ReadPin6
		{
			Regs.u16[RegEncoderCCwSteps]++;		//hit
		}
		else
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}	
	}
	else if (GPIO_Pin == GPIO_PIN_7)
	{
		if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) == GPIO_PIN_RESET)		//Could maybe replace if statment with ccw += ReadPin7 and cw +=  ~ReadPin7
		{
			Regs.u16[RegEncoderCwSteps]++;		//hit
		}
		else
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}
	}*/
	
	/*//Try both PF8 and 9 as a rising/falling edge interrupt
	if (GPIO_Pin == GPIO_PIN_8)
	{
		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9) == GPIO_PIN_RESET)		//Could maybe replace if statment with ccw += ReadPin6 and cw +=  ~ReadPin6
		{
			Regs.u16[RegEncoderCCwSteps]++;	//hit
		}
		else
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}	
	}
	else if (GPIO_Pin == GPIO_PIN_9)
	{
		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8) == GPIO_PIN_RESET)		//Could maybe replace if statment with ccw += ReadPin7 and cw +=  ~ReadPin7
		{
			Regs.u16[RegEncoderCwSteps]++;	//hit
		}
		else
		{
			Regs.u16[RegEncoderCCwSteps]++;
		}
	}*/
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/*Used to count steps until CheckMotorUpdatePwm() should stop the PWM that drives the motor.*/
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {  // If the interrupt is triggered by channel 1
		numMotorStepsCompleted++;
	}
	//This was an attempt at getting encoder timers working but they seem unfeasable on an F7 board with limited bus ports. It is never triggering. Use GPIO interrupt instead. Code from https://www.steppeschool.com/pages/blog/stm32-timer-encoder-mode
	/*
	static int testCounter = -1;
	static float testDegrees = -1.1;
	if (htim->Instance == TIM3)
	{
		testCounter = TIM3->CNT;
		testDegrees = (testCounter - 0x8000) * 360 / 600;
	}
	*/
	//Should probably make this an else if, but I'm not certain that a Callback cannot be called by multiple timers simultaneously. 

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	// measure velocity and position
	UpdateEncoder(htim);                                         
}
/*
 *An attempt to get encoder working by using encoder timers. However, HAL_TIM_IC_CaptureCallback is never getting triggered by TIM3.
 *Following the guide at https://www.steppeschool.com/pages/blog/stm32-timer-encoder-mode
*/
void UpdateEncoder(TIM_HandleTypeDef *htim)             
{
	//	 TIM3->CNT maxes out at TIM3->ARR which is 65,535. Using a 600 steps/rotation encoder this gives greater than 100 rotations worth of steps (combining directions; however, 
	 //  jitter will artificially inflate TIM3->CNT. Essentially there are many false positives. 
	 //  I'm pretty certain that in Encoder mode, the timer both counts up and counts down, so this shouldn't be a problem.
	uint32_t currentCounterValue = __HAL_TIM_GET_COUNTER(htim);
	static uint16_t lastCounterValue;
	static uint8_t first_time = 0;
	
	if (!first_time)
	{
		Regs.u16[RegEncoderVelocity] = 0;
		first_time = 1;
	}
	else
	{
		if (currentCounterValue == lastCounterValue)	//If encoder hasn't moved 
		{
			Regs.u16[RegEncoderVelocity] = 0;
		}
		else if (currentCounterValue > lastCounterValue)
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				Regs.u16[RegEncoderVelocity] = -lastCounterValue - (__HAL_TIM_GET_AUTORELOAD(htim) - currentCounterValue);
			}
			else
			{
				Regs.u16[RegEncoderVelocity] = currentCounterValue - lastCounterValue;
			}
		}
		else
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				Regs.u16[RegEncoderVelocity] = currentCounterValue - lastCounterValue;
			}
			else
			{
				Regs.u16[RegEncoderVelocity] = currentCounterValue + (__HAL_TIM_GET_AUTORELOAD(htim) - lastCounterValue);
			}
		}
	}
	Regs.u16[RegEncoderPosition] += Regs.u16[RegEncoderVelocity];
	lastCounterValue = currentCounterValue;                                         
}
/*Can alter timer period and duty cycle. This is used to alter motor stepping speed.*/
void AdjustPWM(TIM_HandleTypeDef* htim, uint32_t Channel, uint16_t period, uint16_t pulseWidth)
{
	//WARNING: The first pulse will likely have an inconsistent duty cycle. Effectivley--the first rising/falling edge could either be delayed or early by an ammount depending on the previous period.
	__HAL_TIM_SET_AUTORELOAD(htim, period);
	__HAL_TIM_SET_COMPARE(htim, Channel, pulseWidth);
}

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