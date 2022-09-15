/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId LEDTaskHandle;
osThreadId LowPriorityHandle;
osThreadId MidPriorityHandle;
osThreadId HighPriorityHandle;
osMutexId MuxSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void const * argument);
void LowPriorityTask(void const * argument);
void MidPriorityTask(void const * argument);
void HighPriorityTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of MuxSem */
  osMutexDef(MuxSem);
  MuxSemHandle = osMutexCreate(osMutex(MuxSem));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of LowPriority */
  osThreadDef(LowPriority, LowPriorityTask, osPriorityIdle, 0, 128);
  LowPriorityHandle = osThreadCreate(osThread(LowPriority), NULL);

  /* definition and creation of MidPriority */
  osThreadDef(MidPriority, MidPriorityTask, osPriorityIdle, 0, 128);
  MidPriorityHandle = osThreadCreate(osThread(MidPriority), NULL);

  /* definition and creation of HighPriority */
  osThreadDef(HighPriority, HighPriorityTask, osPriorityIdle, 0, 128);
  HighPriorityHandle = osThreadCreate(osThread(HighPriority), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartLEDTask */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
    osDelay(500);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
    osDelay(500);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_LowPriorityTask */
/**
* @brief Function implementing the LowPriority thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LowPriorityTask */
void LowPriorityTask(void const * argument)
{
  /* USER CODE BEGIN LowPriorityTask */
  static uint32_t i;
  osStatus xReturn;
  /* Infinite loop */
  for(;;)
  {
	printf("LowPriority_Task get mutex.\r\n");
	xReturn = osMutexWait(MuxSemHandle, osWaitForever);

	if(osOK == xReturn)
	{
		printf("LowPriority_Task running...\r\n");
	}

	for(i=0;i<2000000;i++)
	{
		taskYIELD();		//发起任务调度
	}

	printf("LowPriority_Task release mutex.\r\n");
	xReturn = osMutexRelease(MuxSemHandle);

    osDelay(1000);
  }
  /* USER CODE END LowPriorityTask */
}

/* USER CODE BEGIN Header_MidPriorityTask */
/**
* @brief Function implementing the MidPriority thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MidPriorityTask */
void MidPriorityTask(void const * argument)
{
  /* USER CODE BEGIN MidPriorityTask */
  /* Infinite loop */
  for(;;)
  {
	printf("MidPriority_Task Running...\r\n");
    osDelay(1000);
  }
  /* USER CODE END MidPriorityTask */
}

/* USER CODE BEGIN Header_HighPriorityTask */
/**
* @brief Function implementing the HighPriority thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HighPriorityTask */
void HighPriorityTask(void const * argument)
{
  /* USER CODE BEGIN HighPriorityTask */
  osStatus xReturn;
  /* Infinite loop */
  for(;;)
  {
	printf("HighPriority_Task get mutex.\r\n");
	xReturn = osMutexWait(MuxSemHandle, osWaitForever);
	if(osOK == xReturn)
	{
		printf("HighPriority_Task Running...\r\n");
	}

	printf("HighPriority_Task release mutex.\r\n");
	xReturn = osMutexRelease(MuxSemHandle);

    osDelay(1000);
  }
  /* USER CODE END HighPriorityTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
