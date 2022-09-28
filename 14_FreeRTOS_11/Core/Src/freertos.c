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
#include <string.h>
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
osThreadId LED1Handle;
osThreadId LED2Handle;
osThreadId CPUHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LED1Task(void const * argument);
void LED2Task(void const * argument);
void CPUTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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

  /* definition and creation of LED1 */
  osThreadDef(LED1, LED1Task, osPriorityIdle, 0, 128);
  LED1Handle = osThreadCreate(osThread(LED1), NULL);

  /* definition and creation of LED2 */
  osThreadDef(LED2, LED2Task, osPriorityIdle, 0, 128);
  LED2Handle = osThreadCreate(osThread(LED2), NULL);

  /* definition and creation of CPU */
  osThreadDef(CPU, CPUTask, osPriorityIdle, 0, 256);
  CPUHandle = osThreadCreate(osThread(CPU), NULL);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LED1Task */
/**
* @brief Function implementing the LED1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED1Task */
void LED1Task(void const * argument)
{
  /* USER CODE BEGIN LED1Task */
  /* Infinite loop */
  for(;;)
  {
	osDelay(500);   /* 延时500个tick */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	printf("LED1_ON\r\n");
	osDelay(500);   /* 延时500个tick */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	printf("LED1_OFF\r\n");
  }
  /* USER CODE END LED1Task */
}

/* USER CODE BEGIN Header_LED2Task */
/**
* @brief Function implementing the LED2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED2Task */
void LED2Task(void const * argument)
{
  /* USER CODE BEGIN LED2Task */
  /* Infinite loop */
  for(;;)
  {
	osDelay(300);   /* 延时300个tick */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	printf("LED2_ON\r\n");
	osDelay(300);   /* 延时300个tick */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	printf("LED2_OFF\r\n");
  }
  /* USER CODE END LED2Task */
}

/* USER CODE BEGIN Header_CPUTask */
/**
* @brief Function implementing the CPU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CPUTask */
void CPUTask(void const * argument)
{
  /* USER CODE BEGIN CPUTask */
  uint8_t CPU_RunInfo[400];		//保存任务运行时间信息
  /* Infinite loop */
  for(;;)
  {
	memset(CPU_RunInfo, 0, 400);

	osThreadList(CPU_RunInfo);

    printf("---------------------------------------------\r\n");
    printf("Task      Task_Status Priority  Remaining_Stack Task_No\r\n");
    printf("%s", CPU_RunInfo);
    printf("---------------------------------------------\r\n");

    memset(CPU_RunInfo, 0, 400);

    vTaskGetRunTimeStats((char *)&CPU_RunInfo);

    printf("Task       Running_Count        Utilization\r\n");
    printf("%s", CPU_RunInfo);
    printf("---------------------------------------------\r\n\n");

    osDelay(1000);
  }
  /* USER CODE END CPUTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
