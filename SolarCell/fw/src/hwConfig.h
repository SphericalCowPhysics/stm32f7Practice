/**
  ******************************************************************************
  * @file           : hwConfig.h
  * @brief          : Code to configure the MCU devices (hardware)
  ******************************************************************************
  */
#ifndef __HWCONFIG_H__
#define __HWCONFIG_H__

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
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim10;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void InitUsb(void);
void MX_SPI2_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
void MX_ADC3_Init(void);
void MX_TIM1_Init(void);
void HAL_MspInit(void);
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm);
	
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);
void MX_TIM10_Init(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);
void MPU_Config(void);
	
#ifdef __cplusplus
}
#endif

#endif // __HWCONFIG_H__