/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
osThreadId ledTaskHandle;
osThreadId buzzerTaskHandle;
osThreadId uartTaskHandle;
osThreadId canTaskHandle;
osThreadId plotTaskHandle;
osThreadId IMU_TaskHandle;
osThreadId Calibrate_TaskHandle;
osThreadId Chassis_L_TaskHandle;
osThreadId Chassis_R_TaskHandle;
osThreadId observeTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void led_task(void const * argument);
extern void buzzer_task(void const * argument);
extern void uart_task(void const * argument);
extern void can_task(void const * argument);
extern void plot_task(void const * argument);
extern void imu_task(void const * argument);
extern void calibrate_task(void const * argument);
extern void chassis_L_task(void const * argument);
extern void chassis_R_task(void const * argument);
extern void observe_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* definition and creation of ledTask */
  osThreadDef(ledTask, led_task, osPriorityIdle, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of buzzerTask */
  osThreadDef(buzzerTask, buzzer_task, osPriorityIdle, 0, 128);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, uart_task, osPriorityIdle, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, can_task, osPriorityIdle, 0, 128);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* definition and creation of plotTask */
  osThreadDef(plotTask, plot_task, osPriorityIdle, 0, 128);
  plotTaskHandle = osThreadCreate(osThread(plotTask), NULL);

  /* definition and creation of IMU_Task */
  osThreadDef(IMU_Task, imu_task, osPriorityHigh, 0, 1024);
  IMU_TaskHandle = osThreadCreate(osThread(IMU_Task), NULL);

  /* definition and creation of Calibrate_Task */
  osThreadDef(Calibrate_Task, calibrate_task, osPriorityBelowNormal, 0, 128);
  Calibrate_TaskHandle = osThreadCreate(osThread(Calibrate_Task), NULL);

  /* definition and creation of Chassis_L_Task */
  osThreadDef(Chassis_L_Task, chassis_L_task, osPriorityAboveNormal, 0, 512);
  Chassis_L_TaskHandle = osThreadCreate(osThread(Chassis_L_Task), NULL);

  /* definition and creation of Chassis_R_Task */
  osThreadDef(Chassis_R_Task, chassis_R_task, osPriorityAboveNormal, 0, 512);
  Chassis_R_TaskHandle = osThreadCreate(osThread(Chassis_R_Task), NULL);

  /* definition and creation of observeTask */
  osThreadDef(observeTask, observe_task, osPriorityHigh, 0, 512);
  observeTaskHandle = osThreadCreate(osThread(observeTask), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
