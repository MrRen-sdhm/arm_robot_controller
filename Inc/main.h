/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define configUSE_NEWLIB_REENTRANT 1
#define LWIP_POSIX_SOCKETS_IO_NAMES 0

#define RS485_1_WE_Pin GPIO_PIN_2
#define RS485_1_WE_GPIO_Port GPIOE
#define LED_STATUS2_Pin GPIO_PIN_4
#define LED_STATUS2_GPIO_Port GPIOE
#define LED_STATUS1_Pin GPIO_PIN_5
#define LED_STATUS1_GPIO_Port GPIOE
#define BEEP_Pin GPIO_PIN_6
#define BEEP_GPIO_Port GPIOE
#define SENSE_TOP_Pin GPIO_PIN_13
#define SENSE_TOP_GPIO_Port GPIOC
#define SENSE_MIDDLE_Pin GPIO_PIN_14
#define SENSE_MIDDLE_GPIO_Port GPIOC
#define SENSE_BOTTOM_Pin GPIO_PIN_15
#define SENSE_BOTTOM_GPIO_Port GPIOC
#define RS485_1_RX_Pin GPIO_PIN_3
#define RS485_1_RX_GPIO_Port GPIOA
#define PWR_DOWN_INT_Pin GPIO_PIN_7
#define PWR_DOWN_INT_GPIO_Port GPIOE
#define LIFT_ALM_Pin GPIO_PIN_8
#define LIFT_ALM_GPIO_Port GPIOE
#define LIFT_DIR_DOWN_Pin GPIO_PIN_9
#define LIFT_DIR_DOWN_GPIO_Port GPIOE
#define LIFT_DISABLE_Pin GPIO_PIN_10
#define LIFT_DISABLE_GPIO_Port GPIOE
#define LIFT_UNLOCK_Pin GPIO_PIN_11
#define LIFT_UNLOCK_GPIO_Port GPIOE
#define SERVO1_Pin GPIO_PIN_13
#define SERVO1_GPIO_Port GPIOE
#define SERVO2_Pin GPIO_PIN_14
#define SERVO2_GPIO_Port GPIOE
#define KEY_DOWN_Pin GPIO_PIN_15
#define KEY_DOWN_GPIO_Port GPIOE
#define KEY_UP_Pin GPIO_PIN_10
#define KEY_UP_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define I2C3_INT2_Pin GPIO_PIN_10
#define I2C3_INT2_GPIO_Port GPIOD
#define I2C3_INT1_Pin GPIO_PIN_11
#define I2C3_INT1_GPIO_Port GPIOD
#define LIFT_PULSE_Pin GPIO_PIN_13
#define LIFT_PULSE_GPIO_Port GPIOD
#define RS485_2_TX_Pin GPIO_PIN_9
#define RS485_2_TX_GPIO_Port GPIOA
#define RS485_2_RX_Pin GPIO_PIN_10
#define RS485_2_RX_GPIO_Port GPIOA
#define RS485_3_WE_Pin GPIO_PIN_15
#define RS485_3_WE_GPIO_Port GPIOA
#define RS485_3_TX_Pin GPIO_PIN_10
#define RS485_3_TX_GPIO_Port GPIOC
#define RS485_3_RX_Pin GPIO_PIN_11
#define RS485_3_RX_GPIO_Port GPIOC
#define RS485_2_WE_Pin GPIO_PIN_3
#define RS485_2_WE_GPIO_Port GPIOD
#define LED_START_Pin GPIO_PIN_4
#define LED_START_GPIO_Port GPIOD
#define RS485_1_TX_Pin GPIO_PIN_5
#define RS485_1_TX_GPIO_Port GPIOD
#define BUT_START_Pin GPIO_PIN_6
#define BUT_START_GPIO_Port GPIOD
#define LED_STOP_Pin GPIO_PIN_7
#define LED_STOP_GPIO_Port GPIOD
#define BUT_STOP_Pin GPIO_PIN_3
#define BUT_STOP_GPIO_Port GPIOB
#define LED_RESET_Pin GPIO_PIN_4
#define LED_RESET_GPIO_Port GPIOB
#define BUT_RESET_Pin GPIO_PIN_5
#define BUT_RESET_GPIO_Port GPIOB
#define I2C1_INT1_Pin GPIO_PIN_8
#define I2C1_INT1_GPIO_Port GPIOB
#define I2C1_INT2_Pin GPIO_PIN_9
#define I2C1_INT2_GPIO_Port GPIOB
#define BUT_EMG1_Pin GPIO_PIN_0
#define BUT_EMG1_GPIO_Port GPIOE
#define BUT_EMG2_Pin GPIO_PIN_1
#define BUT_EMG2_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
