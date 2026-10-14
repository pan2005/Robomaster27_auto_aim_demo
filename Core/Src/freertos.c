/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "bsp_usb_cdc.h"
#include "imu_task.h"
#include "gimbal_task.h"
#include "system_identification_task.h"
#include "velocity_control_test_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
uint32_t KeyTaskBuffer[ 128 ];
osStaticThreadDef_t KeyTaskControlBlock;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .cb_mem = &KeyTaskControlBlock,
  .cb_size = sizeof(KeyTaskControlBlock),
  .stack_mem = &KeyTaskBuffer[0],
  .stack_size = sizeof(KeyTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for LcdTask */
osThreadId_t LcdTaskHandle;
uint32_t LcdTaskBuffer[ 256 ];
osStaticThreadDef_t LcdTaskControlBlock;
const osThreadAttr_t LcdTask_attributes = {
  .name = "LcdTask",
  .cb_mem = &LcdTaskControlBlock,
  .cb_size = sizeof(LcdTaskControlBlock),
  .stack_mem = &LcdTaskBuffer[0],
  .stack_size = sizeof(LcdTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
uint32_t ImuTaskBuffer[ 1024 ];
osStaticThreadDef_t ImuTaskControlBlock;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .cb_mem = &ImuTaskControlBlock,
  .cb_size = sizeof(ImuTaskControlBlock),
  .stack_mem = &ImuTaskBuffer[0],
  .stack_size = sizeof(ImuTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FunTest */
osThreadId_t FunTestHandle;
uint32_t FunTestBuffer[ 128 ];
osStaticThreadDef_t FunTestControlBlock;
const osThreadAttr_t FunTest_attributes = {
  .name = "FunTest",
  .cb_mem = &FunTestControlBlock,
  .cb_size = sizeof(FunTestControlBlock),
  .stack_mem = &FunTestBuffer[0],
  .stack_size = sizeof(FunTestBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void KeyTask_Entry(void *argument);
void LcdTask_Entry(void *argument);
void ImuTask_Entry(void *argument);
void FunTest_Entry(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* Existing application tasks are intentionally disabled for now. */
  // KeyTaskHandle = osThreadNew(KeyTask_Entry, NULL, &KeyTask_attributes);
  // LcdTaskHandle = osThreadNew(LcdTask_Entry, NULL, &LcdTask_attributes);
  // ImuTaskHandle = osThreadNew(ImuTask_Entry, NULL, &ImuTask_attributes);
  // FunTestHandle = osThreadNew(FunTest_Entry, NULL, &FunTest_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  ImuTask_Init();
#if VELOCITY_CONTROL_TEST_MODE
  VelocityControlTestTask_Init();
#elif SYSTEM_IDENTIFICATION_MODE
  SystemIdentificationTask_Init();
#else
  GimbalTask_Init();
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  struct usb_device *usb_dev = usb_get_device();
  usb_dev->Init(usb_dev);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_KeyTask_Entry */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask_Entry */
__weak void KeyTask_Entry(void *argument)
{
  /* USER CODE BEGIN KeyTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KeyTask_Entry */
}

/* USER CODE BEGIN Header_LcdTask_Entry */
/**
* @brief Function implementing the LcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask_Entry */
__weak void LcdTask_Entry(void *argument)
{
  /* USER CODE BEGIN LcdTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LcdTask_Entry */
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void *argument)
{
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask_Entry */
}

/* USER CODE BEGIN Header_FunTest_Entry */
/**
* @brief Function implementing the FunTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FunTest_Entry */
__weak void FunTest_Entry(void *argument)
{
  /* USER CODE BEGIN FunTest_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FunTest_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

