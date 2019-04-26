/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define SWITCH_0_Pin GPIO_PIN_0
#define SWITCH_0_GPIO_Port GPIOC
#define SWITCH_0_EXTI_IRQn EXTI0_IRQn
#define SWITCH_1_Pin GPIO_PIN_1
#define SWITCH_1_GPIO_Port GPIOC
#define SWITCH_1_EXTI_IRQn EXTI1_IRQn
#define SWITCH_2_Pin GPIO_PIN_2
#define SWITCH_2_GPIO_Port GPIOC
#define SWITCH_2_EXTI_IRQn EXTI2_IRQn
#define SWITCH_3_Pin GPIO_PIN_3
#define SWITCH_3_GPIO_Port GPIOC
#define SWITCH_3_EXTI_IRQn EXTI3_IRQn
#define SWITCH_4_Pin GPIO_PIN_4
#define SWITCH_4_GPIO_Port GPIOA
#define SWITCH_4_EXTI_IRQn EXTI4_IRQn
#define DIODA_Pin GPIO_PIN_0
#define DIODA_GPIO_Port GPIOB
#define DS18B20_INPUT_PIN_Pin GPIO_PIN_1
#define DS18B20_INPUT_PIN_GPIO_Port GPIOB
#define DS18B20_OUTPUT_PIN_Pin GPIO_PIN_2
#define DS18B20_OUTPUT_PIN_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_10
#define LCD_D4_GPIO_Port GPIOB
#define SPI2_SS_Pin GPIO_PIN_12
#define SPI2_SS_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOC
#define LCD_D5_Pin GPIO_PIN_8
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_9
#define LCD_D6_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_3
#define LCD_RS_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_4
#define LCD_E_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_5
#define LCD_RW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
