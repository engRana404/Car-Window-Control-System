/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Button_Task(void *pvParameters);
void Driver_Up_Auto(void *pvParameters);
void Driver_Up_Manual(void *pvParameters);
void Driver_Down_Auto(void *pvParameters);
void Driver_Down_Manual(void *pvParameters);
void Passenger_Up_Auto(void *pvParameters);
void Passenger_Up_Manual(void *pvParameters);
void Passenger_Down_Auto(void *pvParameters);
void Passenger_Down_Manual(void *pvParameters);
void Handle_Button_Press(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, xSemaphoreHandle xMutex, int *PressedState, TaskFunction_t autoTask, TaskFunction_t manuTask, TaskHandle_t *autoHandle, TaskHandle_t *manuHandle);
void handleWindowMovement(void (*Motor_Move)(void), volatile bool *limitReached, volatile bool *oppositeLimitReached, volatile int *jamDetected, TaskHandle_t handle);
void Handle_Manual_Movement(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, void (*Motor_Move)(void), volatile bool *limitReached, volatile int *jamDetected, TaskHandle_t handle);
void Motor_Move_Forward(void);
void Motor_Move_Backward(void);
void Set_Upper_Limit(void *pvParameters);
void Set_Lower_Limit(void *pvParameters);
void Set_Window_Lock(void *pvParameters);
void Set_Jam_Detected(void *pvParameters);

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

//Button Debounce and Long Press Delay
#define DEBOUNCE_DELAY_MS 50
#define LONG_PRESS_DELAY_MS 1000

//Port A
#define Driver_Up_Button_Pin        GPIO_PIN_12
#define Driver_Down_Button_Pin      GPIO_PIN_11
//Port B
#define Passenger_Up_Button_Pin     GPIO_PIN_12
#define Passenger_Down_Button_Pin   GPIO_PIN_10
#define Window_Lock_Pin             GPIO_PIN_2
#define Jam_Detected_Pin            GPIO_PIN_1
#define Upper_Limit_Pin             GPIO_PIN_15
#define Lower_Limit_Pin             GPIO_PIN_14
#define Motor_Pin_1                 GPIO_PIN_13
#define Motor_Pin_2		            GPIO_PIN_5

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
