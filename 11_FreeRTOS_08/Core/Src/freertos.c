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
#include "stdio.h"	//引用标准库
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
osThreadId defaultTaskHandle;
osThreadId SendHandle;
osThreadId Receive1Handle;
osThreadId Receive2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void SendTask(void const * argument);
void Receive1Task(void const * argument);
void Receive2Task(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Send */
  osThreadDef(Send, SendTask, osPriorityAboveNormal, 0, 128);
  SendHandle = osThreadCreate(osThread(Send), NULL);

  /* definition and creation of Receive1 */
  osThreadDef(Receive1, Receive1Task, osPriorityNormal, 0, 128);
  Receive1Handle = osThreadCreate(osThread(Receive1), NULL);

  /* definition and creation of Receive2 */
  osThreadDef(Receive2, Receive2Task, osPriorityBelowNormal, 0, 128);
  Receive2Handle = osThreadCreate(osThread(Receive2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
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
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  uint32_t count = 0;
  /* Infinite loop */
  for(;;)
  {
	if(count%10==0)
	{
		xReturn = xTaskNotifyGive(Receive1Handle);
		if(xReturn == pdTRUE)
		{
			printf("Receive1_Task_Handle 任务通知释放成功！\r\n");
		}
	}
	if(count%20==0)
	{
		xReturn = xTaskNotifyGive(Receive2Handle);
		if(xReturn == pdTRUE)
		{
			printf("Receive2_Task_Handle 任务通知释放成功！\r\n");
		}
	}
	count++;
    osDelay(1000);
  }
  /* USER CODE END SendTask */
}

/* USER CODE BEGIN Header_Receive1Task */
/**
* @brief Function implementing the Receive1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Receive1Task */
void Receive1Task(void const * argument)
{
  /* USER CODE BEGIN Receive1Task */
  /* Infinite loop */
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	printf("Receive1_Task 任务通知获取成功！\r\n");
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);		//红灯翻转
  }
  /* USER CODE END Receive1Task */
}

/* USER CODE BEGIN Header_Receive2Task */
/**
* @brief Function implementing the Receive2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Receive2Task */
void Receive2Task(void const * argument)
{
  /* USER CODE BEGIN Receive2Task */
  /* Infinite loop */
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	printf("Receive2_Task 任务通知获取成功！\r\n");
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);		//黄灯翻转
  }
  /* USER CODE END Receive2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
