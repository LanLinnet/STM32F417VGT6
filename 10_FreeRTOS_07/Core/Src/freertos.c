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
static uint32_t TmrCb_Count1 = 0; /* 记录软件定时器 1 回调函数执行次数 */
static uint32_t TmrCb_Count2 = 0; /* 记录软件定时器 2 回调函数执行次数 */
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId AppTaskCreateHandle;
osTimerId Swtmr1Handle;
osTimerId Swtmr2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartAppTaskCreate(void const * argument);
void Swtmr1_Callback(void const * argument);
void Swtmr2_Callback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the timer(s) */
  /* definition and creation of Swtmr1 */
  osTimerDef(Swtmr1, Swtmr1_Callback);
  Swtmr1Handle = osTimerCreate(osTimer(Swtmr1), osTimerPeriodic, NULL);

  /* definition and creation of Swtmr2 */
  osTimerDef(Swtmr2, Swtmr2_Callback);
  Swtmr2Handle = osTimerCreate(osTimer(Swtmr2), osTimerOnce, NULL);

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

  /* definition and creation of AppTaskCreate */
  osThreadDef(AppTaskCreate, StartAppTaskCreate, osPriorityIdle, 0, 128);
  AppTaskCreateHandle = osThreadCreate(osThread(AppTaskCreate), NULL);

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
  printf("我在循环外\r\n");
  uint8_t count = 0;
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    osDelay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    printf("我在循环内 %d\r\n",(int)count);
    count++;
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAppTaskCreate */
/**
* @brief Function implementing the AppTaskCreate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAppTaskCreate */
void StartAppTaskCreate(void const * argument)
{
  /* USER CODE BEGIN StartAppTaskCreate */
  taskENTER_CRITICAL();		//进入临界区

  osTimerStart(Swtmr1Handle, 1000);		//打开软件定时器1
  osTimerStart(Swtmr2Handle, 5000);		//打开软件定时器2

  vTaskDelete(AppTaskCreateHandle); //删除 AppTaskCreate 任务

  taskEXIT_CRITICAL(); //退出临界区
  /* Infinite loop */
  /* USER CODE END StartAppTaskCreate */
}

/* Swtmr1_Callback function */
void Swtmr1_Callback(void const * argument)
{
  /* USER CODE BEGIN Swtmr1_Callback */
	TickType_t tick_num1;
	TmrCb_Count1++;
	tick_num1 = osKernelSysTick();

	printf("swtmr1_callback %d\r\n", (int)TmrCb_Count1);
	printf("tick_num=%d\r\n", (int)tick_num1);
  /* USER CODE END Swtmr1_Callback */
}

/* Swtmr2_Callback function */
void Swtmr2_Callback(void const * argument)
{
  /* USER CODE BEGIN Swtmr2_Callback */
	TickType_t tick_num2;
	TmrCb_Count2++;
	tick_num2 = osKernelSysTick();

	printf("swtmr2_callback %d\r\n", (int)TmrCb_Count2);
	printf("tick_num=%d\r\n", (int)tick_num2);
  /* USER CODE END Swtmr2_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
