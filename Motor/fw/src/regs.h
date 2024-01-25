/**
  ******************************************************************************
  * @file           : regs.h
  * @brief          : Code to handle registers that do not survive power cycle
  ******************************************************************************
  */
#ifndef __REGS_H__
#define __REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"

#define REG_SIZE 512
#define REG_SIZE8      (REG_SIZE-4)
#define REG_SIZE16     (REG_SIZE8/2)
#define REG_SIZE32     (REG_SIZE16/2)

typedef struct REG_BLOCK {
	 // 512 bytes - 4 per page
    uint8_t start;
	uint8_t seq;
	uint8_t ver;
	uint8_t chksum;
	union {
		uint8_t u8[REG_SIZE8];
		uint16_t u16[REG_SIZE16];
		int16_t s16[REG_SIZE16];
		uint32_t u32[REG_SIZE32];
		int32_t s32[REG_SIZE32];
		float f32[REG_SIZE32];
	};
} REG_BLOCK;

extern volatile REG_BLOCK Regs;
	
enum SYS_STATUS {
  RegFirmWareVersion = 0,      //0, Version # set at top of main.c
  RegUniqueID,                 //1, a unique ID for each MCU (96 bits, but only returns lowest 16 bits)
  RegTick,                     //2, lower 16 bits of the 1mS system timer
  RegAdcTemp,                  //3, read temperature sensor internal to MCU, 0.76V @25degC + 2.5mV/degC, temp = 25+(nAdc*3.3/4095-0.76)/0.025, 1022 => 27.5degC
  RegAdcRef,                   //4, MCU internal reference voltage (1.2Vnom), should be around 1.2/3.3*4095 = 1500, can use to calculate voltage of 3.3V supply
  RegImuWhoAmI,                //5, Who Am I register from IMU
  RegImuAx,                    //6, accelerometer X value
  RegImuAy,                    //7, accelerometer Y value
  RegImuAz,                    //8, accelerometer Z value
  RegLast
};

void InitRegs();
uint8_t UpdateRegs();
uint8_t ReadReg(uint16_t nReg);
void SetReg(uint16_t nReg, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif // __REGS_H__