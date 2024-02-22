/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
#define MotorPwmCounter_Pin GPIO_PIN_15
#define MotorPwmCounter_GPIO_Port GPIOA
#define MotorPwm_Pin GPIO_PIN_8
#define MotorPwm_GPIO_Port GPIOA
#define EncoderB_Pin GPIO_PIN_7
#define EncoderB_GPIO_Port GPIOC
#define EncoderA_Pin GPIO_PIN_6
#define EncoderA_GPIO_Port GPIOC	
	
#define MotorPWM_EventsPer2ms 8000/2

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "regs.h"
#include "nvParams.h"
#include "commands.h"
	
//#define EvalKit 1	
//#define BitBangMotor
//#define InterruptTesting	
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

extern 	uint32_t motorPulseStartTime;
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
