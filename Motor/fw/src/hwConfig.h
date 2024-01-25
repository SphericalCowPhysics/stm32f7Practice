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

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void InitUsb(void);
void MX_SPI2_Init(void);
void MX_ADC1_Init(void);

#ifdef __cplusplus
}
#endif

#endif // __HWCONFIG_H__