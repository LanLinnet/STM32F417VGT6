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
osThreadId ReceiveHandle;
osThreadId SendHandle;
osSemaphoreId BinarySemHandle;
osSemaphoreId CountingSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void const * argument);
void ReceiveTask(void const * argument);
void SendTask(void const * argument);

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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem */
  osSemaphoreDef(BinarySem);
  BinarySemHandle = osSemaphoreCreate(osSemaphore(BinarySem), 1);

  /* definition and creation of CountingSem */
  osSemaphoreDef(CountingSem);
  CountingSemHandle = osSemaphoreCreate(osSemaphore(CountingSem), 5);

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

  /* definition and creation of Receive */
  osThreadDef(Receive, ReceiveTask, osPriorityIdle, 0, 128);
  ReceiveHandle = osThreadCreate(osThread(Receive), NULL);

  /* definition and creation of Send */
  osThreadDef(Send, SendTask, osPriorityIdle, 0, 128);
  SendHandle = osThreadCreate(osThread(Send), NULL);

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
__weak void StartLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
	//printf("LED Task!\r\n");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(1000);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_ReceiveTask */
/**
* @brief Function implementing the Receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveTask */
void ReceiveTask(void const * argument)
{
  /* USER CODE BEGIN ReceiveTask */
  osStatus xReturn = osErrorValue;  //定义一个创建信息返回值，默认为osErrorValue - value of a parameter is out of range.
  /* Infinite loop */
  for(;;)
  {
	  //二值信号量
	xReturn = osSemaphoreWait(BinarySemHandle, osWaitForever);
	if(osOK == xReturn)
	{
		printf("BinarySem get!\r\n");
	}

//	  //计数信号量
//	  xReturn = osSemaphoreWait(CountingSemHandle, 0);
//	  if(osOK == xReturn)
//	  {
//		  printf("Successfully applied for parking space.\r\n");
//	  }
//	  else
//	  {
//		  printf("Sorry, the parking lot is full now!\r\n");
//	  }

    osDelay(1000);
  }
  /* USER CODE END ReceiveTask */
}

/* USER CODE BEGIN Header_SendTask */
/**
* @brief Function implementing the Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendTask */
void SendTask(void const * argument)
{
  /* USER CODE BEGIN SendTask */
  osStatus xReturn;
  uint8_t count=0;
  /* Infinite loop */
  for(;;)
  {
	if(count==10)
	{
		count = 0;
		//二值信号量
		xReturn = osSemaphoreRelease(BinarySemHandle);
		if(osOK == xReturn)
		{
			printf("Release!\r\n");
		}
		else
		{
			printf("BinarySem release failed!\r\n");
		}
//		//计数信号量
//		xReturn = osSemaphoreRelease(CountingSemHandle);
//		if(osOK == xReturn)
//		{
//			printf("Release 1 parking space!\r\n");
//		}
//		else
//		{
//			printf("There is no parking space to release!\r\n");
//		}
	}
	count++;
    osDelay(500);
  }
  /* USER CODE END SendTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
