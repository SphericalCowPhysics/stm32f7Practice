/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"

/* External variables --------------------------------------------------------*/
//extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
//C:\MicroProjects\McuMotor\fw\src\UsbCdc_STM32F7xx\usbd_conf.c

extern PCD_HandleTypeDef hpcd;
extern TIM_HandleTypeDef htim2;
//C:\MicroProjects\McuMotor\fw\src\stm32f7xx_it.c


/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  __ASM volatile("BKPT #01");
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}
/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{

	//HAL_TIM_IRQHandler(&htim2);
	
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  * Use this interrupt to toggle green LED on PI1 using User Button on PI11 as the interrupt trigger
  */
void EXTI15_10_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */

	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

void EXTI9_5_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */

	/* USER CODE END EXTI9_5_IRQn 0 */
	
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
	
	/* USER CODE BEGIN EXTI9_5_IRQn 1 */

	/* USER CODE END EXTI9_5_IRQn 1 */
}
