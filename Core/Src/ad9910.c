/*
 * ad9910.c
 *
 *  Created on: Aug 8, 2021
 *      Author: hansg
 */

#include "ad9910.h"		//SX1276 header file
#include <stdint.h>
#include "main.h"
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include "retarget.h"

#define DEBUG_PRINTING

static uint8_t nbytes[] = {4,4,4,4,4,4,4,2,4,4,8,8,8,4,8,8,8,8,8,8,8,4}; //register lengths in bytes

state AD9910_Init(){
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET); //deselect SPI

	#ifdef DEBUG_PRINTING
	printf("Initializing AD9910\n\r");
	#endif

	state rs = PASS;
	rs |= AD9910_Reset(); //reset the AD9910

	rs |= AD9910_WriteReg(CFR1, (SDIO_INPUT_ONLY));
	rs |= AD9910_IO_Update();

	rs |= AD9910_WriteReg(CFR2, (SYNC_TIM_V_DIS | A_FROM_STP));
	rs |= AD9910_IO_Update();

	rs |= AD9910_WriteReg(CFR3, (VCO5 | CP_387 | INP_DIV_RST | PLL_EN | (PLL_MULT << 1))); //refclk off
	rs |= AD9910_IO_Update();

	rs |= AD9910_SetProfile(PF0); //set profile
	HAL_GPIO_WritePin(AD9910_TXE_GPIO_Port, AD9910_TXE_Pin, GPIO_PIN_SET);  //enable TX

	return rs;
}

state AD9910_ConfigureRamp(float lower, float upper, float ramptime){

	#ifdef DEBUG_PRINTING
	printf("Configuring Ramp from %fHz to %fHz in %f seconds\n\r", lower, upper, ramptime);
	#endif

	state rs = PASS;
	HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_RESET); //don't ramp
	uint32_t upper_ftw = FREQ_TO_FTW(upper);
	uint32_t lower_ftw = FREQ_TO_FTW(lower);
	uint32_t upstep_ftw = ((upper_ftw - lower_ftw) / ramptime) * (4.0f / FSYSCLK);
	uint16_t p = 1;
	if(upstep_ftw < 1) {  //if we're stepping so fast that the steps need to be smaller
		upstep_ftw = 1;
		p = FSYSCLK/4 * ramptime / (upper_ftw - lower_ftw);
	}

	#ifdef DEBUG_PRINTING
	printf("\tUpper FTW: %d\n\r\tLower FTW: %d\n\r\tPos Freq Step FTW: %d\n\r\tP: %d\n\r", (int) upper_ftw, (int) lower_ftw, (int) upstep_ftw, (int) p);
	#endif

	rs |= AD9910_WriteReg(DR_LIMIT, (uint64_t) upper_ftw << 32 | lower_ftw);
	rs |= AD9910_WriteReg(DR_STEP_SIZE, (uint64_t) upstep_ftw << 32 | upstep_ftw);
	rs |= AD9910_WriteReg(DR_RATE, (uint32_t) p << 16 | p);
	AD9910_IO_Update();

	//Turn on DRG, set destination to frequency
	#ifdef DEBUG_PRINTING
	printf("Turning on DRG\n\r");
	#endif
	uint32_t cfr2 = AD9910_ReadReg32(CFR2);
	rs |= AD9910_WriteReg(CFR2, ((cfr2 & ~(0b11 << 20)) | DR_ENABLE | DR_NDH));
	rs |= AD9910_IO_Update();
	return rs;
}

state AD9910_StartRamp(){
	state rs = PASS;
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET); //let the system know we're about to ramp
	HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_SET); //ramp

	while(!HAL_GPIO_ReadPin(AD9910_DROVER_GPIO_Port, AD9910_DROVER_Pin)); //wait for
	HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_RESET); //don't ramp
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	return rs;
}

state AD9910_SingleTone(uint8_t profile, float frequency, float amplitude){
	#ifdef DEBUG_PRINTING
	printf("Setting up PF%d with %fHz at %f volts\n\r", profile, frequency, amplitude);
	#endif

	//TODO, check bounds for frequency and amplitude
	state rs = PASS;
	uint16_t asf = AMP_TO_ASF(amplitude); //compute the amplitude scale factor
	uint32_t ftw = FREQ_TO_FTW(frequency);                             //compute the frequency tuning word

	#ifdef DEBUG_PRINTING
	printf("\tASF: %d\n\r\tFTW: %d\n\r", (int) asf, (int) ftw);
	#endif

	rs |= AD9910_WriteReg(0x0E + profile, (((uint64_t) asf << 48) | (ftw))); //load those parameters into the profile
	rs |= AD9910_IO_Update();

	HAL_GPIO_WritePin(AD9910_TXE_GPIO_Port, AD9910_TXE_Pin, GPIO_PIN_SET);  //enable TX

	rs |= AD9910_SetProfile(profile); //set profile

	return rs;
}

state AD9910_Reset(){
	#ifdef DEBUG_PRINTING
	printf("AD9910 Master Reset\n\r");
	#endif

	HAL_GPIO_WritePin(AD9910_RST_GPIO_Port, AD9910_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(AD9910_RST_GPIO_Port, AD9910_RST_Pin, GPIO_PIN_RESET);
	return PASS;
}

state AD9910_WriteReg(uint8_t reg, uint64_t value){

	#ifdef DEBUG_PRINTING
	printf("Writing value  0x%x%x to register 0x%x\n\r", (unsigned int) (value >> 32), (unsigned int) (value & 0xFFFFFFFF), (unsigned int) reg);
	#endif

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);
	uint8_t payloadsize = nbytes[reg];
	uint8_t packet[payloadsize + 1];
	packet[0] = reg;

	for(int i = 0; i < payloadsize; i++){ 				//fill the packet
		packet[i+1] = ((value >> ((payloadsize - 1 - i) * 8)) & 0xFF);
	}

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, packet, payloadsize + 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	return PASS; //TODO - read the contents of the register out again to check for pass/fail
}

uint64_t AD9910_ReadReg64(uint8_t reg){

	#ifdef DEBUG_PRINTING
	printf("Reading from register 0x%x, ", reg);
	#endif

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	if(nbytes[reg] != 8) return FAIL;
	uint8_t res[8];
	uint8_t readreg = reg | 0x80;

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &readreg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, res, 8, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	uint64_t result = ((uint64_t) res[0] << 56 |
				(uint64_t) res[1] << 48 |
				(uint64_t) res[2] << 40 |
				(uint64_t) res[3] << 32 |
				(uint64_t) res[4] << 24 |
				(uint64_t) res[5] << 16 |
				(uint64_t) res[6] << 8 |
				(uint64_t) res[7]);

	#ifdef DEBUG_PRINTING
	printf("Result: 0x%x%x\n\r", (int) (result >> 32), (int) (result & 0xFFFFFFFF));
	#endif

	return result;
}

uint32_t AD9910_ReadReg32(uint8_t reg){
	#ifdef DEBUG_PRINTING
	printf("Reading from register 0x%x, ", reg);
	#endif

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	if(nbytes[reg] != 4) return FAIL;
	uint8_t res[4];
	uint8_t readreg = reg | 0x80;

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &readreg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, res, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	uint32_t result = ((uint32_t) res[0] << 24 |
			(uint32_t) res[1] << 16 |
			(uint32_t) res[2] << 8 |
			(uint32_t) res[3]);

	#ifdef DEBUG_PRINTING
	printf("Result: 0x%x\n\r", (unsigned int) result);
	#endif

	return result;
}

uint16_t AD9910_ReadReg16(uint8_t reg){
	#ifdef DEBUG_PRINTING
	printf("Reading from register 0x%x, ", reg);
	#endif

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	if(nbytes[reg] != 2) return FAIL;
	uint8_t res[2];
	uint8_t readreg = reg | 0x80;

	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &readreg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, res, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	uint16_t result = ((uint16_t) res[0] << 8 |
			(uint16_t) res[1]);

	#ifdef DEBUG_PRINTING
	printf("Result: 0x%x\n\r", result);
	#endif

	return result;
}

state AD9910_IO_Update(){
	#ifdef DEBUG_PRINTING
	printf("-- IO Update --\n\r");
	#endif

	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	return PASS;
}

state AD9910_SetProfile(uint8_t pf){												//TODO: should this be synchronous?
	#ifdef DEBUG_PRINTING
	printf("Setting Profile to PF%d\n\r", pf);
	#endif

	HAL_GPIO_WritePin(AD9910_PF0_GPIO_Port, AD9910_PF0_Pin, pf & 0x01);
	HAL_GPIO_WritePin(AD9910_PF1_GPIO_Port, AD9910_PF1_Pin, (pf >> 1) & 0x01);
	HAL_GPIO_WritePin(AD9910_PF2_GPIO_Port, AD9910_PF2_Pin, (pf >> 2) & 0x01);
	return PASS;
}

