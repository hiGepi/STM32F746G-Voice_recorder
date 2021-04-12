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
#include "stm32f7xx_hal.h"

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
#define LCD_B0_Pin GPIO_PIN_4
#define LCD_B0_GPIO_Port GPIOE
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define LCD_B1_Pin GPIO_PIN_13
#define LCD_B1_GPIO_Port GPIOJ
#define LCD_DE_Pin GPIO_PIN_7
#define LCD_DE_GPIO_Port GPIOK
#define LCD_B7_Pin GPIO_PIN_6
#define LCD_B7_GPIO_Port GPIOK
#define LCD_B6_Pin GPIO_PIN_5
#define LCD_B6_GPIO_Port GPIOK
#define LCD_B4_Pin GPIO_PIN_12
#define LCD_B4_GPIO_Port GPIOG
#define LCD_B2_Pin GPIO_PIN_14
#define LCD_B2_GPIO_Port GPIOJ
#define LCD_HSYNC_Pin GPIO_PIN_10
#define LCD_HSYNC_GPIO_Port GPIOI
#define LCD_B5_Pin GPIO_PIN_4
#define LCD_B5_GPIO_Port GPIOK
#define LCD_BL_CTRL_Pin GPIO_PIN_3
#define LCD_BL_CTRL_GPIO_Port GPIOK
#define LCD_B3_Pin GPIO_PIN_15
#define LCD_B3_GPIO_Port GPIOJ
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define LCD_DISP_Pin GPIO_PIN_12
#define LCD_DISP_GPIO_Port GPIOI
#define LCD_VSYNC_Pin GPIO_PIN_9
#define LCD_VSYNC_GPIO_Port GPIOI
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define LCD_G6_Pin GPIO_PIN_1
#define LCD_G6_GPIO_Port GPIOK
#define LCD_G7_Pin GPIO_PIN_2
#define LCD_G7_GPIO_Port GPIOK
#define OSC_25M_Pin GPIO_PIN_0
#define OSC_25M_GPIO_Port GPIOH
#define LCD_INT_Pin GPIO_PIN_13
#define LCD_INT_GPIO_Port GPIOI
#define LCD_R0_Pin GPIO_PIN_15
#define LCD_R0_GPIO_Port GPIOI
#define LCD_G4_Pin GPIO_PIN_11
#define LCD_G4_GPIO_Port GPIOJ
#define LCD_G5_Pin GPIO_PIN_0
#define LCD_G5_GPIO_Port GPIOK
#define LCD_CLK_Pin GPIO_PIN_14
#define LCD_CLK_GPIO_Port GPIOI
#define LCD_G1_Pin GPIO_PIN_8
#define LCD_G1_GPIO_Port GPIOJ
#define LCD_G3_Pin GPIO_PIN_10
#define LCD_G3_GPIO_Port GPIOJ
#define LCD_G0_Pin GPIO_PIN_7
#define LCD_G0_GPIO_Port GPIOJ
#define LCD_G2_Pin GPIO_PIN_9
#define LCD_G2_GPIO_Port GPIOJ
#define LCD_R7_Pin GPIO_PIN_6
#define LCD_R7_GPIO_Port GPIOJ
#define LCD_R5_Pin GPIO_PIN_4
#define LCD_R5_GPIO_Port GPIOJ
#define LCD_R6_Pin GPIO_PIN_5
#define LCD_R6_GPIO_Port GPIOJ
#define LCD_R4_Pin GPIO_PIN_3
#define LCD_R4_GPIO_Port GPIOJ
#define LCD_R3_Pin GPIO_PIN_2
#define LCD_R3_GPIO_Port GPIOJ
#define LCD_R1_Pin GPIO_PIN_0
#define LCD_R1_GPIO_Port GPIOJ
#define LCD_R2_Pin GPIO_PIN_1
#define LCD_R2_GPIO_Port GPIOJ
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
