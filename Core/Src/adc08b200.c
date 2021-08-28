/*
 * adc08b200.c
 *
 *  Created on: Aug 11, 2021
 *      Author: hansg
 */

#include "adc08b200.h"
#include <stdint.h>
#include "main.h"
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include "retarget.h"

#define ADC_DEBUG_PRINTING

uint8_t ADC_PrintBuf(uint8_t databuf[]){
	printf(PREFIX);

	for(int i = 0; i < BSIZE; i++){
		printf("%d,", databuf[i]);
	}

	#ifdef ADC_DEBUG_PRINTING
	printf("\n\n\r");
	#endif

	return NICE; //  ;]
}

uint8_t ADC_Init(){
	#ifdef ADC_DEBUG_PRINTING
	printf("Initializing the ADC08b200\n\r");
	#endif

	//wake up
	HAL_GPIO_WritePin(ADC_PD_GPIO_Port, ADC_PD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADC_PDADC_GPIO_Port, ADC_PDADC_Pin, GPIO_PIN_RESET);

	//set Auto-Stop Write
	HAL_GPIO_WritePin(ADC_ASW_GPIO_Port, ADC_ASW_Pin, GPIO_PIN_SET);

	//set OEDGE to low, data transitions on falling edge of DRDY
	HAL_GPIO_WritePin(ADC_OEDGE_GPIO_Port,ADC_OEDGE_Pin, GPIO_PIN_RESET);

	//turn on the clock
	HAL_GPIO_WritePin(ADC_CLKEN_GPIO_Port,ADC_CLKEN_Pin, GPIO_PIN_SET);

	//enable outputs
	HAL_GPIO_WritePin(ADC_OE_GPIO_Port, ADC_OE_Pin, GPIO_PIN_SET);

	ADC_Reset();
	ADC_GetStatus();

	return NICE;
}

// reads out the internal ADC buffer into RAM with CPU
uint8_t ADC_ReadBufferCPU(uint8_t data[]){
	#ifdef ADC_DEBUG_PRINTING
	printf("Reading the ADC Buffer with CPU\n\r");
	#endif


	//warn me
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);

	if(!HAL_GPIO_ReadPin(ADC_FF_GPIO_Port, ADC_FF_Pin)){
		#ifdef ADC_DEBUG_PRINTING
		printf("\treadBuffer() failed, buffer is not full\n\r");
		#endif
		return RIP;
	}

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	/*
	 * See Figure 6 in the datasheet for this timing
	 */
	HAL_GPIO_WritePin(ADC_REN_GPIO_Port, ADC_REN_Pin, GPIO_PIN_SET);

	for(int i = 0; i < 2; i++){  //TODO timing constraing violationion
		pulseRCLK();
		HAL_Delay(1);
	}

	for(int i = 0; i < BSIZE; i++){		//read out the data
		while(!HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin)); //wait for the data
		data[i] = ((GPIOD->IDR) >> 8);
		pulseRCLK();
	}

	HAL_GPIO_WritePin(ADC_REN_GPIO_Port, ADC_REN_Pin, GPIO_PIN_RESET);

	return NICE;

}

//helper function for reading the buffer
void pulseRCLK(){
	HAL_GPIO_WritePin(ADC_RCLK_GPIO_Port, ADC_RCLK_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ADC_RCLK_GPIO_Port, ADC_RCLK_Pin, GPIO_PIN_RESET);
}

/*
 * Fills the buffer with samples at Fs
 */
uint8_t ADC_FillBuffer(){
	#ifdef ADC_DEBUG_PRINTING
	printf("Filling the ADC Buffer\n\r");
	#endif

	if(!HAL_GPIO_ReadPin(ADC_EF_GPIO_Port, ADC_EF_Pin)){
		#ifdef ADC_DEBUG_PRINTING
		printf("\tFillBuffer() failed, buffer is not empty\n\r");
		#endif
		return RIP;
	}

	//assert WEN to start a capture
	HAL_GPIO_WritePin(ADC_WEN_GPIO_Port, ADC_WEN_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	while(!HAL_GPIO_ReadPin(ADC_FF_GPIO_Port, ADC_FF_Pin)); //wait for the buffer to be full
	HAL_GPIO_WritePin(ADC_WEN_GPIO_Port, ADC_WEN_Pin, GPIO_PIN_RESET); //stop writing
	return NICE;
}

//really only useful for printing the status. Oh well
uint8_t ADC_GetStatus(){
	#ifdef ADC_DEBUG_PRINTING
	printf("ADC Status:\n\r");

	uint8_t ff = HAL_GPIO_ReadPin(ADC_FF_GPIO_Port, ADC_FF_Pin);
	uint8_t ef = HAL_GPIO_ReadPin(ADC_FF_GPIO_Port, ADC_EF_Pin);
	uint8_t drdy = HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin);

	printf("\tFull Flag: %d\n\r", ff);
	printf("\tEmpty Flag: %d\n\r", ef);
	printf("\tData Ready: %d\n\r", drdy);
	#endif

	return (ff << 2 | ef << 1 | drdy << 0);
}

uint8_t ADC_FullPowerDown(){
	#ifdef ADC_DEBUG_PRINTING
	printf("Powering down ADC\n\r");
	#endif

	HAL_GPIO_WritePin(ADC_PD_GPIO_Port, ADC_PD_Pin, GPIO_PIN_SET);
	return NICE;
}

uint8_t ADC_Reset(){
	#ifdef ADC_DEBUG_PRINTING
	printf("Resetting the ADC\n\r");
	#endif

	HAL_GPIO_WritePin(ADC_PD_GPIO_Port, ADC_PD_Pin, GPIO_PIN_RESET); //don't be off
	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_RESET);
	return NICE;
}


