/*
 * adc08b200.h
 *
 *  Created on: Aug 11, 2021
 *      Author: hansg
 */

#ifndef INC_ADC08B200_H_
#define INC_ADC08B200_H_

	#include <stdint.h>
	#include "stm32f4xx.h"

	//status return macros, courtesy of Eli
	#define NICE 0
	#define RIP 1

	//device constants
	#define MULT 8  //clock frequency multiplier, as set by the MULT pins
	#define BSIZE 1024 //buffer size, as set by the BSIZE pins

	//offsets for getStatus
	#define FF (1 << 2)
	#define EF (1 << 1)
	#define DRDY (1 << 0)

	//function prototypes
	uint8_t ADC_Init();
	uint8_t ADC_ReadBufferCPU(uint8_t data[]);
	uint8_t ADC_ReadBufferDMA(uint8_t data[]);
	void pulseRCLK();
	uint8_t ADC_FillBuffer();
	uint8_t ADC_GetStatus();
	uint8_t ADC_FullPowerDown();
	uint8_t ADC_Reset();

#endif /* INC_ADC08B200_H_ */
