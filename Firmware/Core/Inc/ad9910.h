/*
 * ad9910.h
 *
 *  Created on: Aug 8, 2021
 *      Author: hansg
 */

#ifndef INC_AD9910_H_
#define INC_AD9910_H_

	#include <stdint.h>
	#include "stm32f4xx.h"

	typedef enum state {FAIL = 1, PASS = 0} state; //status return type

	extern SPI_HandleTypeDef hspi2;

	//DDS Constants
	#define PLL_MULT 40				//pll multiplier, should this not be 25?
	#define FULL_SCALE_I_OUT 0.02007				//datasheet page 23. Iout = 86.4/RSET*(1+CODE/96)
	#define LOAD_IMPEDANCE 50
	#define FSYSCLK 1000000000.0f
	#define RAM_LENGTH 1024LL

	//function prototypes
	state AD9910_Init();
	state AD9910_Reset();
	state AD9910_WriteReg(uint8_t reg, uint64_t value);
	state AD9910_IO_Update();
	state AD9910_SingleTone(uint8_t profile, float frequency, float amplitude);
	state AD9910_SetProfile(uint8_t pf);
	state AD9910_ConfigureRamp(float lower, float upper, float ramptime);
	state AD9910_StartRamp();
	uint64_t AD9910_ReadReg64(uint8_t reg);
	uint32_t AD9910_ReadReg32(uint8_t reg);
	uint16_t AD9910_ReadReg16(uint8_t reg);
	state AD9910_StartRAMRamp();
	state AD9910_ConfigureRAM(float ramptime);
	state AD9910_ConfigureDefaultFreq(float frequency);
	state AD9910_ConfigureChirp(float lower, float upper, float duration);
	state AD9910_Chirp();


	//register macros
	#define CFR1 0x00
	#define CFR2 0x01
	#define CFR3 0x02
	#define AUX_DAC_CTRL 0x03
	#define IO_UPDATE_RATE 0x04
	#define FTW 0x07
	#define POW 0x08
	#define ASF 0x09
	#define MULTICHIP_SYNC 0x0A
	#define DR_LIMIT 0x0B
	#define DR_STEP_SIZE 0x0C
	#define DR_RATE 0x0D
	#define STP0 0x0E
	#define RP0 0x0E
	#define STP1 0x0F
	#define RP1 0x0F
	#define STP2 0x10
	#define RP2 0x10
	#define STP3 0x11
	#define RP3 0x11
	#define STP4 0x12
	#define RP4 0x12
	#define STP5 0x13
	#define RP5 0x13
	#define STP6 0x14
	#define RP6 0x14
	#define STP7 0x15
	#define RP8 0x15
	#define RAM 0x16

	//function macros
	#define FREQ_TO_FTW(x) (uint32_t) lroundf(((x * (1LL << 32)) / FSYSCLK))
	#define AMP_TO_ASF(x) (uint16_t) lroundf(((x * (1 << 14)) / (LOAD_IMPEDANCE * FULL_SCALE_I_OUT)))

	//bit macros
	#define SDIO_INPUT_ONLY (1 << 1)
	#define A_FROM_STP (1 << 24)
	#define SYNC_TIM_V_DIS (1 << 5)
	#define VCO1 (0b001 << 24)
	#define VCO5 (0b101 << 24)
	#define CP_387 (7 << 19)
	#define INP_DIV_RST (1 << 14)
	#define PLL_EN (1 << 8)
	#define DR_ENABLE (1 << 19)
	#define DR_NDH (1 << 18)
	#define DR_NDL (1 << 17)
	#define RPMC_RAMP_UP (0b001 << 0)
	#define RPMC_RAMP_UPDOWN (0b010 << 0)
	#define RAM_ENABLE (1 << 31)
	#define RAM_DEST_AMP (0b10 << 29)
	#define DR_CLEAR (1 << 12)
	#define PHASE_CLEAR (1 << 11)

	//profiles
	#define PF0 0
	#define PF1 1
	#define PF2 2
	#define PF3 3
	#define PF4 4
	#define PF5 5
	#define PF6 6
	#define PF7 7

#endif /* INC_AD9910_H_ */
