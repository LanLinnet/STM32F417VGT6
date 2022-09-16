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
#define EVENT_2 (0x01 << 0)	//设置事件掩码的位 0
#define EVENT_3 (0x01 << 1)	//设置事件掩码的位 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Send */
osThreadId_t SendHandle;
const osThreadAttr_t Send_attributes = {
  .name = "Send",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Receive */
osThreadId_t ReceiveHandle;
const osThreadAttr_t Receive_attributes = {
  .name = "Receive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Event */
osEventFlagsId_t EventHandle;
const osEventFlagsAttr_t Event_attributes = {
  .name = "Event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void *argument);
void SendTask(void *argument);
void ReceiveTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of Send */
  SendHandle = osThreadNew(SendTask, NULL, &Send_attributes);

  /* creation of Receive */
  ReceiveHandle = osThreadNew(ReceiveTask, NULL, &Receive_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Event */
  EventHandle = osEventFlagsNew(&Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLEDTask */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDTask */
__weak void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_SendTask */
/**
* @brief Function implementing the Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendTask */
void SendTask(void *argument)
{
  /* USER CODE BEGIN SendTask */
  uint8_t count=1;
  /* Infinite loop */
  for(;;)
  {
	if(count%10==0)
	{
		printf("EVENT_2 Send!\r\n");
		osEventFlagsSet(EventHandle, EVENT_2);
	}
	if(count%20==0)
	{
		printf("EVENT_3 Send!\r\n");
		osEventFlagsSet(EventHandle, EVENT_3);
	}
	count++;
    osDelay(500);
  }
  /* USER CODE END SendTask */
}

/* USER CODE BEGIN Header_ReceiveTask */
/**
* @brief Function implementing the Receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveTask */
void ReceiveTask(void *argument)
{
  /* USER CODE BEGIN ReceiveTask */
  uint32_t r_event;  //定义事件接收变量
  /* Infinite loop */
  for(;;)
  {
	 r_event= osEventFlagsWait(EventHandle, EVENT_2|EVENT_3, osFlagsWaitAll, osWaitForever);
	//如果接收完成并且正确
	if((r_event & (EVENT_2|EVENT_3)) == (EVENT_2|EVENT_3))
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
		osDelay(1000);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
		printf("EVENT_2 | EVENT_3 Done!\r\n");
	}
	else
	{
		printf("Event Error!\r\n");
	}
    osDelay(1000);
  }
  /* USER CODE END ReceiveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

