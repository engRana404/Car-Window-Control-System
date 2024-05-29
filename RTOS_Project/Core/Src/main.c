/*
 * main.c
 *
 *  Created on: May 28, 2024
 *      Author: Rana Gamal
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
static volatile int jamDetected = 0;
static volatile int windowLock = 0;
static volatile bool upperLimitReached = false;
static volatile bool lowerLimitReached = false;
static volatile bool isPassenger = false;

// Task handles
TaskHandle_t xdrivUpAutoHandle, xdrivUpManuHandle, xdrivDownAutoHandle, xdrivDownManuHandle;
TaskHandle_t xpassUpAutoHandle, xpassUpManuHandle, xpassDownAutoHandle, xpassDownManuHandle;
TaskHandle_t xTaskHandlePassed;

// Semaphores
xSemaphoreHandle xupperLimitReachedSemaphore, xlowerLimitReachedSemaphore;
xSemaphoreHandle xwindowLockSemaphore, xjamDetectedSemaphore;

// Mutexes
xSemaphoreHandle xDriverUpButtonMutex, xDriverDownButtonMutex;

// Queue
xQueueHandle xQueue;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Main program entry point */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Initialize mutexes */
  xDriverUpButtonMutex = xSemaphoreCreateMutex();
  xDriverDownButtonMutex = xSemaphoreCreateMutex();

  /* Initialize semaphores */
  xupperLimitReachedSemaphore = xSemaphoreCreateBinary();
  xlowerLimitReachedSemaphore = xSemaphoreCreateBinary();
  xwindowLockSemaphore = xSemaphoreCreateBinary();
  xjamDetectedSemaphore = xSemaphoreCreateBinary();

  /* Initialize queue */
  xQueue = xQueueCreate(1, sizeof(TaskHandle_t));

  /* Create tasks */
  createTasks();

  HAL_GPIO_WritePin(GPIOB, Motor_Pin_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, Motor_Pin_2, GPIO_PIN_SET);

  vTaskStartScheduler();

  while (1)
  {
    // Infinite loop
  }
}

void createTasks(void) {
  if (xupperLimitReachedSemaphore && xlowerLimitReachedSemaphore &&
      xwindowLockSemaphore && xjamDetectedSemaphore) {

    xTaskCreate(Set_Upper_Limit, "setupper", 256, NULL, 4, NULL);
    xTaskCreate(Set_Lower_Limit, "setlower", 256, NULL, 4, NULL);
    xTaskCreate(Set_Window_Lock, "setwindowlock", 256, NULL, 4, NULL);
    xTaskCreate(Set_Jam_Detected, "setjamdetected", 256, NULL, 4, NULL);
    xTaskCreate(Button_Task, "Button Task", 256, NULL, 1, NULL);
  }
}

void Button_Task(void *pvParameters) {
    // Define variables to keep track of button states and times
    TickType_t xLastWakeTime;
    const TickType_t xDelay = pdMS_TO_TICKS(1); // delay of 1 ms
    xLastWakeTime = xTaskGetTickCount();

    int Driver_Up_Pressed = 0, Driver_Down_Pressed = 0;
    int Passenger_Up_Pressed = 0, Passenger_Down_Pressed = 0;

    while (1) {
        Handle_Button_Press(GPIOA, Driver_Up_Button_Pin, xDriverUpButtonMutex,
                          &Driver_Up_Pressed, Driver_Up_Auto, Driver_Up_Manual,
                          &xdrivUpAutoHandle, &xdrivUpManuHandle);
        Handle_Button_Press(GPIOA, Driver_Down_Button_Pin, xDriverDownButtonMutex,
                          &Driver_Down_Pressed, Driver_Down_Auto, Driver_Down_Manual,
                          &xdrivDownAutoHandle, &xdrivDownManuHandle);
        Handle_Button_Press(GPIOB, Passenger_Up_Button_Pin, NULL,
                          &Passenger_Up_Pressed, Passenger_Up_Auto, Passenger_Up_Manual,
                          &xpassUpAutoHandle, &xpassUpManuHandle);
        Handle_Button_Press(GPIOB, Passenger_Down_Button_Pin, NULL,
                          &Passenger_Down_Pressed, Passenger_Down_Auto, Passenger_Down_Manual,
                          &xpassDownAutoHandle, &xpassDownManuHandle);
    }
}

void Handle_Button_Press(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                       xSemaphoreHandle xMutex, int *PressedState,
                       TaskFunction_t autoTask, TaskFunction_t manuTask,
                       TaskHandle_t *autoHandle, TaskHandle_t *manuHandle) {
    if (xMutex) xSemaphoreTake(xMutex, portMAX_DELAY);
    if (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
        if (!(*PressedState)) {
            *PressedState = 1;
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
            if (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
                TickType_t xLastWakeTime = xTaskGetTickCount();
                while ((!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                if ((!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                    xTaskCreate(manuTask, "ManualTask", 128, NULL, 2, manuHandle);
                } else {
                    xTaskCreate(autoTask, "AutoTask", 128, NULL, 2, autoHandle);
                }
            }
            *PressedState = 0;
        }
    }
    if (xMutex) xSemaphoreGive(xMutex);
}


/* DRIVER TASKS */
void Driver_Up_Auto(void *pvParameters) {
	isPassenger = false;
    Handle_Window_Movement(Motor_Move_Forward, &upperLimitReached, &lowerLimitReached, &jamDetected, xdrivUpAutoHandle);
}

void Driver_Up_Manual(void *pvParameters) {
	isPassenger = false;
    Handle_Manual_Movement(GPIOA, Driver_Up_Button_Pin, Motor_Move_Forward, &upperLimitReached, &jamDetected, xdrivUpManuHandle);
}

void Driver_Down_Auto(void *pvParameters) {
	isPassenger = false;
    Handle_Window_Movement(Motor_Move_Backward, &lowerLimitReached, &upperLimitReached, &jamDetected, xdrivDownAutoHandle);
}

void Driver_Down_Manual(void *pvParameters) {
	isPassenger = false;
    Handle_Manual_Movement(GPIOA, Driver_Down_Button_Pin, Motor_Move_Backward, &lowerLimitReached, &jamDetected, xdrivDownManuHandle);
}

/* PASSENGER TASKS */
void Passenger_Up_Auto(void *pvParameters) {
	isPassenger = true;
    Handle_Window_Movement(Motor_Move_Forward, &upperLimitReached, &lowerLimitReached, &jamDetected, xpassUpAutoHandle);
}

void Passenger_Up_Manual(void *pvParameters) {
	isPassenger = true;
    Handle_Manual_Movement(GPIOB, Passenger_Up_Button_Pin, Motor_Move_Forward, &upperLimitReached, &jamDetected, xpassUpManuHandle);
}

void Passenger_Down_Auto(void *pvParameters) {
	isPassenger = true;
    Handle_Window_Movement(Motor_Move_Backward, &lowerLimitReached, &upperLimitReached, &jamDetected, xpassUpAutoHandle);
}

void Passenger_Down_Manual(void *pvParameters) {
	isPassenger = true;
    Handle_Manual_Movement(GPIOB, Passenger_Down_Button_Pin, Motor_Move_Backward, &lowerLimitReached, &jamDetected, xpassDownManuHandle);
}

/* UTILITY FUNCTIONS */
void Handle_Window_Movement(void (*Motor_Move)(void), volatile bool *limitReached, volatile bool *oppositeLimitReached, volatile int *jamDetected, TaskHandle_t handle) {
    TaskHandle_t xReceivedHandle;
    TickType_t xLastWakeTime;

    if (xQueueReceive(xQueue, &xReceivedHandle, 0) == pdTRUE) {
        vTaskDelete(xReceivedHandle);
    }

    // Check if window lock is activated
    if (windowLock == 1 && isPassenger) {
        // Window lock is activated, stop movement
        Motor_Stop();
        vTaskDelete(handle);
        return;
    }

    while (!(*limitReached) && (*jamDetected == 0)) {
        Motor_Move();
        *oppositeLimitReached = false;
    }

    if(*jamDetected == 1){
		Motor_Stop();
		xLastWakeTime =  xTaskGetTickCount();
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			Motor_Move_Backward();
		}
		*jamDetected = 0;
		Motor_Stop();
		vTaskDelete(handle);
		return;
	}

    Motor_Stop();
    vTaskDelete(handle);
}

void Handle_Manual_Movement(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, void (*Motor_Move)(void), volatile bool *limitReached, volatile int *jamDetected, TaskHandle_t handle) {
    TaskHandle_t xReceivedHandle;
    TickType_t xLastWakeTime;

    if (xQueueReceive(xQueue, &xReceivedHandle, 0) == pdTRUE) {
        vTaskDelete(xReceivedHandle);
    }

    // Check if window lock is activated
    if (windowLock == 1 && isPassenger) {
        // Window lock is activated, stop movement
        Motor_Stop();
        vTaskDelete(handle);
        return;
    }

    while (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) && !(*limitReached) && (*jamDetected == 0)) {
        Motor_Move();
    }

	if(*jamDetected == 1){
		Motor_Stop();
		xLastWakeTime =  xTaskGetTickCount();
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			Motor_Move_Backward();
		}
		*jamDetected = 0;
		Motor_Stop();
		vTaskDelete(handle);
		return;
	}

    Motor_Stop();
    vTaskDelete(handle);
}


void Set_Upper_Limit(void *pvParameters)
{
	// Wait for the semaphore
	while(1){
		xSemaphoreTake(xupperLimitReachedSemaphore, portMAX_DELAY);//semaphore is given in ISR
		//change upper limit flag to true to indicate upper limit reached
		upperLimitReached = true;
	}
}


void Set_Lower_Limit(void *pvParameters)
{
	// Wait for the semaphore
	while(1){
		xSemaphoreTake(xlowerLimitReachedSemaphore, portMAX_DELAY);//semaphore is given in ISR
		//change lower limit flag to true to indicate lower limit reached
		lowerLimitReached = true;
	}
}

void Set_Window_Lock(void *pvParameters)
{
	// Wait for the semaphore
	while(1){
		xSemaphoreTake(xwindowLockSemaphore, portMAX_DELAY);//semaphore is given in ISR
		//toggle window lock accordingly
		windowLock = !windowLock;
	}
}


void Set_Jam_Detected(void *pvParameters)
{
	while(1){
		xSemaphoreTake(xjamDetectedSemaphore, portMAX_DELAY);//semaphore is given in ISR
		//change jam detected flag to 1 to indicate jam was detected and initiate jamming protocol
		jamDetected = 1;
	}

}

void Motor_Move_Forward(void) {
    HAL_GPIO_WritePin(GPIOB, Motor_Pin_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, Motor_Pin_2, GPIO_PIN_RESET);
}

void Motor_Move_Backward(void) {
    HAL_GPIO_WritePin(GPIOB, Motor_Pin_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, Motor_Pin_2, GPIO_PIN_SET);
}

void Motor_Stop(void) {
	HAL_GPIO_WritePin(GPIOB, Motor_Pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Motor_Pin_2, GPIO_PIN_RESET);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
