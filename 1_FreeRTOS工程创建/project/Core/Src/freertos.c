/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "usart.h"
#include <string.h>
#include "BH1750/bsp_bh1750.h"
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
int dis_data = 0;
uint8_t DATA_BUF[8] = { 0 };
/* USER CODE END Variables */
osThreadId BH1750_TaskHandle;
osThreadId DHT11_TASKHandle;
osMessageQId myQueue01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void BH1750Task(void const * argument);
void DHT11Task(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of BH1750_Task */
  osThreadDef(BH1750_Task, BH1750Task, osPriorityNormal, 0, 128);
  BH1750_TaskHandle = osThreadCreate(osThread(BH1750_Task), NULL);

  /* definition and creation of DHT11_TASK */
  osThreadDef(DHT11_TASK, DHT11Task, osPriorityNormal, 0, 128);
  DHT11_TASKHandle = osThreadCreate(osThread(DHT11_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_BH1750Task */
/**
  * @brief  Function implementing the BH1750_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_BH1750Task */
void BH1750Task(void const * argument)
{
  /* USER CODE BEGIN BH1750Task */
	float LIGH = 0;
	uint8_t opecode;
	HAL_Delay(200);
	Init_BH1750();
  /* Infinite loop */
  for(;;)
  {
		opecode = 0x01;
		if ( I2C_BH1750_Opecode_Write(&opecode, 1) != HAL_OK){
			printf("I2C_BH1750_Opecode_Write: %2x Error\n", opecode);
			continue;
		}
		opecode = 0x10;
		if ( I2C_BH1750_Opecode_Write(&opecode, 1) != HAL_OK){
			printf("I2C_BH1750_Opecode_Write: %2x Error\n", opecode);
			continue;
		}
		HAL_Delay(200);	
		if ( I2C_BH1750_Data_Read(DATA_BUF, 2) != HAL_OK){
			printf("I2C_BH1750_Data_Read:  Error\n");
			continue;
		}
		dis_data =(DATA_BUF[0]);
		dis_data=(dis_data<<8)+ (DATA_BUF[1]);//合成数据
		LIGH=(float)dis_data/1.2;
		printf("data: %f lx\r\n", LIGH);
		osDelay(1);
  }
  /* USER CODE END BH1750Task */
}

/* USER CODE BEGIN Header_DHT11Task */
/**
* @brief Function implementing the DHT11_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DHT11Task */
void DHT11Task(void const * argument)
{
  /* USER CODE BEGIN DHT11Task */
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DHT11Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
