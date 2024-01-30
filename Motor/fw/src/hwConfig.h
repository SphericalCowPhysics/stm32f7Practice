/**
  ******************************************************************************
  * @file           : hwConfig.h
  * @brief          : Code to configure the MCU devices (hardware)
  ******************************************************************************
  */
#ifndef __HWCONFIG_H__
#define __HWCONFIG_H__
#define Encoder2_Pin GPIO_PIN_5
#define Encoder2_GPIO_Port GPIOB
#define Encoder1_Pin GPIO_PIN_4
#define Encoder1_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_2
#define DIR_GPIO_Port GPIOI

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
extern TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void InitUsb(void);
void MX_SPI2_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void Error_Handler(void);	
#ifdef __cplusplus
}
#endif

#endif // __HWCONFIG_H__