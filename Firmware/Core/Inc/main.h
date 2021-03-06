/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_1
#define SW1_GPIO_Port GPIOA
#define SW3B_Pin GPIO_PIN_4
#define SW3B_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOA
#define SW3A_Pin GPIO_PIN_6
#define SW3A_GPIO_Port GPIOA
#define ADC_CLKEN_Pin GPIO_PIN_0
#define ADC_CLKEN_GPIO_Port GPIOB
#define ADC_RESET_Pin GPIO_PIN_7
#define ADC_RESET_GPIO_Port GPIOE
#define ADC_DRDY_Pin GPIO_PIN_9
#define ADC_DRDY_GPIO_Port GPIOE
#define ADC_PDADC_Pin GPIO_PIN_10
#define ADC_PDADC_GPIO_Port GPIOE
#define ADC_WENSYNC_Pin GPIO_PIN_11
#define ADC_WENSYNC_GPIO_Port GPIOE
#define ADC_PD_Pin GPIO_PIN_12
#define ADC_PD_GPIO_Port GPIOE
#define ADC_OEDGE_Pin GPIO_PIN_13
#define ADC_OEDGE_GPIO_Port GPIOE
#define ADC_REN_Pin GPIO_PIN_14
#define ADC_REN_GPIO_Port GPIOE
#define ADC_OE_Pin GPIO_PIN_15
#define ADC_OE_GPIO_Port GPIOE
#define ADC_ASW_Pin GPIO_PIN_11
#define ADC_ASW_GPIO_Port GPIOB
#define ADC_WEN_Pin GPIO_PIN_12
#define ADC_WEN_GPIO_Port GPIOB
#define ADC_EF_Pin GPIO_PIN_13
#define ADC_EF_GPIO_Port GPIOB
#define ADC_RCLK_Pin GPIO_PIN_14
#define ADC_RCLK_GPIO_Port GPIOB
#define ADC_FF_Pin GPIO_PIN_15
#define ADC_FF_GPIO_Port GPIOB
#define DATA_Pin GPIO_PIN_8
#define DATA_GPIO_Port GPIOD
#define DATAD9_Pin GPIO_PIN_9
#define DATAD9_GPIO_Port GPIOD
#define DATAD10_Pin GPIO_PIN_10
#define DATAD10_GPIO_Port GPIOD
#define DATAD11_Pin GPIO_PIN_11
#define DATAD11_GPIO_Port GPIOD
#define DATAD12_Pin GPIO_PIN_12
#define DATAD12_GPIO_Port GPIOD
#define DATAD13_Pin GPIO_PIN_13
#define DATAD13_GPIO_Port GPIOD
#define DATAD14_Pin GPIO_PIN_14
#define DATAD14_GPIO_Port GPIOD
#define DATAD15_Pin GPIO_PIN_15
#define DATAD15_GPIO_Port GPIOD
#define AD9910_CS_Pin GPIO_PIN_6
#define AD9910_CS_GPIO_Port GPIOC
#define AD9910_RST_Pin GPIO_PIN_7
#define AD9910_RST_GPIO_Port GPIOC
#define AD9910_IO_RST_Pin GPIO_PIN_8
#define AD9910_IO_RST_GPIO_Port GPIOC
#define AD9910_PWRDN_Pin GPIO_PIN_9
#define AD9910_PWRDN_GPIO_Port GPIOC
#define SYNC_ERR_Pin GPIO_PIN_8
#define SYNC_ERR_GPIO_Port GPIOA
#define AD9910_PF0_Pin GPIO_PIN_9
#define AD9910_PF0_GPIO_Port GPIOA
#define AD9910_RSOVER_Pin GPIO_PIN_10
#define AD9910_RSOVER_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define AD9910_PF1_Pin GPIO_PIN_15
#define AD9910_PF1_GPIO_Port GPIOA
#define AD9910_DROVER_Pin GPIO_PIN_10
#define AD9910_DROVER_GPIO_Port GPIOC
#define AD9910_PF2_Pin GPIO_PIN_11
#define AD9910_PF2_GPIO_Port GPIOC
#define AD9910_PDCLK_Pin GPIO_PIN_12
#define AD9910_PDCLK_GPIO_Port GPIOC
#define AD9910_OSK_Pin GPIO_PIN_0
#define AD9910_OSK_GPIO_Port GPIOD
#define AD9910_PLLLOCK_Pin GPIO_PIN_1
#define AD9910_PLLLOCK_GPIO_Port GPIOD
#define AD9910_IO_UPDATE_Pin GPIO_PIN_2
#define AD9910_IO_UPDATE_GPIO_Port GPIOD
#define AD9910_SYNC_CLK_Pin GPIO_PIN_3
#define AD9910_SYNC_CLK_GPIO_Port GPIOD
#define AD9910_TXE_Pin GPIO_PIN_4
#define AD9910_TXE_GPIO_Port GPIOD
#define AD9910_F0_Pin GPIO_PIN_6
#define AD9910_F0_GPIO_Port GPIOD
#define AD9910_F1_Pin GPIO_PIN_3
#define AD9910_F1_GPIO_Port GPIOB
#define AD9910_DRCTL_Pin GPIO_PIN_5
#define AD9910_DRCTL_GPIO_Port GPIOB
#define AD9910_DRHOLD_Pin GPIO_PIN_7
#define AD9910_DRHOLD_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_1
#define TRIG_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
