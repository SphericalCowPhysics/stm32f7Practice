/**
  ******************************************************************************
  * @file           : hwConfig.h
  * @brief          : Code to configure the MCU devices (hardware)
  ******************************************************************************
  */
#ifndef __HWCONFIG_H__
#define __HWCONFIG_H__
/*
#define Encoder2_Pin GPIO_PIN_5
#define Encoder2_GPIO_Port GPIOB
#define Encoder1_Pin GPIO_PIN_4
#define Encoder1_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_2
#define DIR_GPIO_Port GPIOI
*/
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <usbd_core.h>
#include <usbd_cdc.h>
#include "usbd_cdc_if.h"
#include <usbd_desc.h>

extern PCD_HandleTypeDef hpcd;
extern USBD_HandleTypeDef USBD_Device;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void InitUsb(void);
void MX_TIM1_Init(void);	
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void HAL_MspInit(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
	
#ifdef DefaultFunction
	
void MX_SPI2_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
	
#endif
	
void Error_Handler(void);	
#ifdef __cplusplus
}
#endif

#endif // __HWCONFIG_H__