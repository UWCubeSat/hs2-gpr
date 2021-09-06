/*
 * gpr.c
 *
 *  Created on: Aug 11, 2021
 *      Author: hansg
 */

/*
 * chirps into a length of coax, using the same chirp to de-chirp. Saves
 * data with the ADC, pipes it back over UART. This functionality could be
 * implemented with the other functions in the AD9910 and ADC drivers, but
 * this eliminates the risk of race conditions and timing violations
*/

#include "ad9910.h"
#include "adc08b200.h"
#include "main.h"
#include "retarget.h"
#include <stdio.h>

#define GPR_DEBUG_PRINTING			//debug output enable

uint8_t adcdata[1024]; //adc data buffer

uint8_t GPR_CoaxTest(float flow, float fhigh, float tchirp){
	#ifdef GPR_DEBUG_PRINTING
	printf("Starting a Coaxial Cable Test\n\r");
	printf("Set RANGE SELECT to SPLIT\n\r");
	printf("Disable PA BYPASS\n\r");
	printf("Disable both LNAs\n\r");
	#endif

	//transmitting into the splitter, not directly into the mixer. Set U6 to RF2v
	HAL_GPIO_WritePin(SW1_GPIO_Port,SW1_Pin,GPIO_PIN_SET);

	//Enable receiving - U17 RFC to RF2, U16 RF1 to RF2
	HAL_GPIO_WritePin(SW1_GPIO_Port,SW2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(SW1_GPIO_Port,SW3A_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(SW1_GPIO_Port,SW3A_Pin,GPIO_PIN_RESET);

	//set up the AD9910
	AD9910_ConfigureChirp(flow, fhigh, tchirp);

	//get ready to start the DDS
	uint32_t cfr1 = AD9910_ReadReg32(CFR1);
	AD9910_WriteReg(CFR1, (cfr1 | DR_CLEAR) & ~(RAM_ENABLE));

	AD9910_IO_Update(); //save this

	//arm
	cfr1 = AD9910_ReadReg32(CFR1);
	AD9910_WriteReg(CFR1, ((cfr1 & ~(DR_CLEAR)) | RAM_ENABLE));

	HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_SET); //ramp up when you can

	HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin, GPIO_PIN_SET);  //start the trigger


	//start an ADC read
	HAL_GPIO_WritePin(ADC_WEN_GPIO_Port, ADC_WEN_Pin, GPIO_PIN_SET);

	//start the chirp
	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_SET); //issue an IO update to start the chirp

	//stop reading
	while(!(HAL_GPIO_ReadPin(AD9910_DROVER_GPIO_Port, AD9910_DROVER_Pin) &&
			HAL_GPIO_ReadPin(AD9910_RSOVER_GPIO_Port, AD9910_RSOVER_Pin))); //wait for the ramp to finish
	while(!HAL_GPIO_ReadPin(ADC_FF_GPIO_Port, ADC_FF_Pin)); //wait for the buffer to be full
	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADC_WEN_GPIO_Port, ADC_WEN_Pin, GPIO_PIN_RESET); //stop writing

	HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin, GPIO_PIN_RESET);  //stop the trigger

	//read out the data
	ADC_ReadBufferCPU(adcdata);
	//ADC_PrintBuf(adcdata);

	return NICE;
}


