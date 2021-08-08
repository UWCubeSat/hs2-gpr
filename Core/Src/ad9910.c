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

state AD9910_Init(){
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET); //deselect SPI

	state rs = PASS;
	rs |= AD9910_Reset(); //reset the AD9910

	rs |= AD9910_WriteReg(CFR1, (SDIO_INPUT_ONLY));
	rs |= AD9910_IO_Update();

	rs |= AD9910_WriteReg(CFR2, (A_FROM_STP | SYNC_TIM_V_DIS));
	rs |= AD9910_IO_Update();

	rs |= AD9910_WriteReg(CFR3, (VCO5 | CP_387 | INP_DIV_RST | PLL_EN | PLL_MULT)); //refclk off
	rs |= AD9910_IO_Update();

	rs |= AD9910_SetProfile(PF0); //set profile

	return rs;
}

state AD9910_SingleTone(uint8_t profile, float frequency, float amplitude){
	//TODO, check bounds for frequency and amplitude
	state rs = PASS;
	uint16_t asf = (uint16_t) lroundf(((amplitude * (1 << 14)) / (LOAD_IMPEDANCE * FULL_SCALE_I_OUT))); //compute the amplitude scale factor
	uint32_t ftw = (uint32_t) lroundf(((frequency * ((uint64_t) 1 << 32)) / FSYSCLK));                             //compute the frequency tuning word

	rs |= AD9910_WriteReg(0x0E + profile, (((uint64_t) asf << 48) | (ftw))); //load those parameters into the profile
	rs |= AD9910_IO_Update();

	HAL_GPIO_WritePin(AD9910_TXE_GPIO_Port, AD9910_TXE_Pin, GPIO_PIN_SET);  //enable TX

	return rs;
}

state AD9910_Reset(){
	HAL_GPIO_WritePin(AD9910_RST_GPIO_Port, AD9910_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(AD9910_RST_GPIO_Port, AD9910_RST_Pin, GPIO_PIN_RESET);
	return PASS;
}

state AD9910_WriteReg(uint8_t reg, uint64_t value){
	HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET);
	uint8_t nbytes[] = NBYTE_ARR;
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

state AD9910_IO_Update(){
	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(AD9910_IO_UPDATE_GPIO_Port, AD9910_IO_UPDATE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	return PASS;
}

state AD9910_SetProfile(uint8_t pf){												//TODO: should this be synchronous?
	HAL_GPIO_WritePin(AD9910_PF0_GPIO_Port, AD9910_PF0_Pin, pf & 0x01);
	HAL_GPIO_WritePin(AD9910_PF1_GPIO_Port, AD9910_PF1_Pin, (pf >> 1) & 0x01);
	HAL_GPIO_WritePin(AD9910_PF2_GPIO_Port, AD9910_PF2_Pin, (pf >> 2) & 0x01);
	return PASS;
}

