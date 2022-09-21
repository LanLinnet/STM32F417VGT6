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
uint8_t *Test_Ptr = NULL;
/* USER CODE END Variables */
osThreadId LEDTaskHandle;
osThreadId TestTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void const * argument);
void StartTestTask(void const * argument);

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
  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of TestTask */
  osThreadDef(TestTask, StartTestTask, osPriorityBelowNormal, 0, 128);
  TestTaskHandle = osThreadCreate(osThread(TestTask), NULL);

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
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
    osDelay(1);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartTestTask */
/**
* @brief Function implementing the TestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestTask */
void StartTestTask(void const * argument)
{
  /* USER CODE BEGIN StartTestTask */
  uint32_t g_memsize;
  uint32_t count = 0;
  /* Infinite loop */
  for(;;)
  {
	if(count%10==0)
	{
		if(NULL == Test_Ptr)
		{
			g_memsize = xPortGetFreeHeapSize();	//��ȡ��ǰ�ڴ��С
			printf("ϵͳ��ǰ�ڴ�Ϊ%d�ֽڣ���ʼ�����ڴ�\r\n", (int)g_memsize);
			Test_Ptr = pvPortMalloc(1024);
			if(NULL != Test_Ptr)
			{
				printf("�ڴ�����ɹ���\r\n");
				printf("���뵽���ڴ��ַΪ%#x\r\n",(int)Test_Ptr);

				//��ȡ��ǰʣ���ڴ��С
				g_memsize = xPortGetFreeHeapSize();
				printf("ϵͳ��ǰ�ڴ�ʣ��Ϊ%d�ֽڣ�\r\n", (int)g_memsize);
				//��Test_Ptr��д�����ݣ���ǰϵͳʱ��
				sprintf((char*)Test_Ptr, "��ǰϵͳTickCount = %d\r\n", (int)xTaskGetTickCount());
				printf("д��������� %s \r\n", (char*)Test_Ptr);
			}
		}
		else
		{
			printf("�����ͷ��ڴ�������\r\n");
		}
	}
	if(count%20==0)
	{
		if(NULL != Test_Ptr)
		{
			printf("�ͷ��ڴ棡\r\n");
			vPortFree(Test_Ptr);
			Test_Ptr = NULL;

			g_memsize = xPortGetFreeHeapSize();
			printf("ϵͳ��ǰ�ڴ�Ϊ%d�ֽڣ��ڴ��ͷ����\r\n", (int)g_memsize);
		}
		else
		{
			printf("���������ڴ����ͷ�\r\n");
		}
	}
	count++;
    osDelay(1000);
  }
  /* USER CODE END StartTestTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
