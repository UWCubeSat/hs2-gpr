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
#include "hann.h"

#define DEBUG_PRINTING

//Addresses ---------------0,1,2,3,4,5,6,7,8,9,a,b,c,d,e,f,10,11,12,13,14,15,16
static uint8_t nbytes[] = {4,4,4,4,4,4,4,4,2,4,4,8,8,4,8,8,8, 8, 8, 8, 8, 8, 4}; //register lengths in bytes

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

//Starts the chirp
state AD9910_Chirp(){
	#ifdef DEBUG_PRINTING
	printf("Chirping\n\r");
	#endif
	state rs = PASS;

	/*set the clear digital ramp accumulator bit so that we can start both the DRG and RAM Sweep
	* with an IO Update. I also clear the phase accumulator, so we always start in the same place */
	uint32_t cfr1 = AD9910_ReadReg32(CFR1);
	rs |= AD9910_WriteReg(CFR1, (cfr1 | DR_CLEAR) & ~(RAM_ENABLE));

	AD9910_IO_Update(); //save this

	/*clear the "clear digital ramp accumulator" and "clear phase accumulator" bits so that when the next
	* IO update is issued, both the RAM Sweep and DRG start */
	cfr1 = AD9910_ReadReg32(CFR1);
	rs |= AD9910_WriteReg(CFR1, ((cfr1 & ~(DR_CLEAR)) | RAM_ENABLE));

	HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_SET); //ramp up when you can
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET); //set the trigger
	AD9910_IO_Update(); //issue an IO update to start the chirp
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); //release the trigger
	while(!(HAL_GPIO_ReadPin(AD9910_DROVER_GPIO_Port, AD9910_DROVER_Pin) &&
			HAL_GPIO_ReadPin(AD9910_RSOVER_GPIO_Port, AD9910_RSOVER_Pin))); //wait for the ramp to finish

	HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_RESET); //clear the trigger
	return PASS;
}

/*
 *Sets up a Hann-Windowed Chirp. Only works for sufficiently long durations to use the entire RAM
 */
state AD9910_ConfigureChirp(float lower, float upper, float duration){
	#ifdef DEBUG_PRINTING
	printf("Setting up a Hann-Windowed Chirp from %fHz to %fHz in %fs\n\r", lower, upper, duration);
	#endif

	state rs = PASS;

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); //clear the trigger

	AD9910_ConfigureRamp(lower, upper, duration); //set up the frequency sweep

	AD9910_ConfigureRAM(duration);

	return rs;

}

state AD9910_StartRAMRamp(){
	#ifdef DEBUG_PRINTING
	printf("Starting a RAM Ramp\n\r");
	#endif

	state rs = PASS;
	rs |= AD9910_SetProfile(PF0); 	//set the profile to PF0
	//enable the RAM
	uint32_t cfr1 = AD9910_ReadReg32(CFR1);
	rs |= AD9910_WriteReg(CFR1, (cfr1 | RAM_ENABLE));

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET); //let the system know we're about to ramp
	rs |= AD9910_IO_Update(); //this causes the ramp to start
	while(!HAL_GPIO_ReadPin(AD9910_RSOVER_GPIO_Port, AD9910_RSOVER_Pin)); //wait for ramp to finish
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	return rs;
}

//loads the predistorted Hann window into RAM, and sets up the RAM for playback
//TODO: get this to work even if we can't use the entire RAM
state AD9910_ConfigureRAM(float ramptime){
	#ifdef DEBUG_PRINTING
	printf("Configuring RAM with a %f second predistorted Hann window\n\r", ramptime);
	#endif

	state rs= PASS;
	AD9910_SetProfile(PF0);

	//calculate the address step rate
	uint16_t M0 = (ramptime / RAM_LENGTH) * FSYSCLK / 4;
	uint16_t pf_end_addr = RAM_LENGTH-1;

	if(M0 < 1) {  //not enough time to step through the whole RAM
		M0 = 1;
		pf_end_addr = (uint16_t) (ramptime * (FSYSCLK / 4));
	}

	#ifdef DEBUG_PRINTING
	printf("\tAddress Step Rate: %d\n\r\tAddress Range: 0 to %d\n\r", M0, pf_end_addr);
	#endif

	//program RP0, addresses span the entire RAM, ramp-up mode, no zero crossing, dwell-high
	rs |= AD9910_WriteReg(RP0, ((((uint64_t) M0) << 40) | (((uint64_t) pf_end_addr) << 30) | RPMC_RAMP_UP));

	rs |= AD9910_IO_Update(); //TODO do I want this here?

	//fill in the RAM
	uint8_t ram_addr = RAM;
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &ram_addr, 1, HAL_MAX_DELAY);
	for(int i = 0; i < RAM_LENGTH; i++){
		uint8_t byte1 = (window[i] >> 6) & 0xFF;
		uint8_t byte2 = (window[i] << 2) & 0xFF;
		uint8_t byte3 = 0;
		uint8_t byte4 = 0;
		HAL_SPI_Transmit(&hspi2, &byte1, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi2, &byte2, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi2, &byte3, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi2, &byte4, 1, HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);

	rs |= AD9910_IO_Update();

	//set the RAM destination to amplitude
	uint32_t cfr1 = AD9910_ReadReg32(CFR1);
	rs |= AD9910_WriteReg(CFR1, ((cfr1 & ~(0b11 << 29)) | RAM_DEST_AMP));
	rs |= AD9910_IO_Update();

	return rs;
}

state AD9910_ConfigureDefaultFreq(float frequency){
	#ifdef DEBUG_PRINTING
	printf("Setting up defaults with %fHz\n\r", frequency);
	#endif

	//TODO, check bounds for frequency and amplitude
	state rs = PASS;
	uint32_t ftw = FREQ_TO_FTW(frequency);                             //compute the frequency tuning word

	#ifdef DEBUG_PRINTING
	printf("\tFTW: %d\n\r",(int) ftw);
	#endif

	rs |= AD9910_WriteReg(FTW, ftw);
	rs |= AD9910_IO_Update(); //issue an IO update to latch the data

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
	#ifdef DEBUG_PRINTING
	printf("Triggering a DRG Ramp\n\r");
	#endif

	state rs = PASS;
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET); //let the system know we're about to ramp
	HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_SET); //ramp

	while(!HAL_GPIO_ReadPin(AD9910_DROVER_GPIO_Port, AD9910_DROVER_Pin)); //wait for finish
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

	rs |= AD9910_WriteReg(STP0 + profile, (((uint64_t) asf << 48) | (ftw))); //load those parameters into the profile
	rs |= AD9910_IO_Update();

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

// ----------------------- DEBUG ONLY --------------------------------
//	AD9910_IO_Update(); --- THIS HAS BEEN DISABLED BECAUSE IT TRIGGERS A RAM RAMP WHEN I DON'T WANT IT
//
//	//check to see if the value was written correctly
//	uint64_t nvalue;
//	switch(payloadsize){
//		case 2:
//			nvalue = AD9910_ReadReg16(reg);
//			if(nvalue != value){
//				#ifdef DEBUG_PRINTING
//				printf("\tWrite Fail, value 0x%x\n\r", (unsigned int) value);
//				#endif
//				return FAIL;
//			}
//			break;
//		case 4:
//			nvalue = AD9910_ReadReg32(reg);
//			if(nvalue != value){
//				#ifdef DEBUG_PRINTING
//				printf("\tWrite Fail, value 0x%x\n\r", (unsigned int) value);
//				#endif
//				return FAIL;
//			}
//			break;
//		default:
//			nvalue = AD9910_ReadReg64(reg);
//			if(nvalue != value){
//				#ifdef DEBUG_PRINTING
//				printf("\tWrite Fail, value 0x%x%x\n\r", (unsigned int) (nvalue >> 32), (unsigned int) (nvalue & 0xFFFFFFFF));
//				#endif
//				return FAIL;
//			}
//			break;
//	}
//
//	#ifdef DEBUG_PRINTING
//	printf("\tWrite pass\n\r");
//	#endif

	return PASS;
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

	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	#ifdef DEBUG_PRINTING				//this has to go after, otherwise it delays the trigger
	printf("-- IO Update --\n\r");
	#endif

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

