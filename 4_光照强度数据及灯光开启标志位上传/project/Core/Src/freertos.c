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
#include "ESP/bsp_esp01s.h"
#include "ONENET/onenet.h"
#include "MQTT/MqttKit.h"
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
float LIGH_OneNET_buf;
uint8_t LEDS_OneNET_buf;
/* USER CODE END Variables */
osThreadId BH1750_TaskHandle;
osThreadId LED_TASKHandle;
osThreadId INFO_TaskHandle;
osThreadId OneNETTaskHandle;
osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
osTimerId myTimer01Handle;
osSemaphoreId LED_FLAG_SemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void BH1750Task(void const * argument);
void LEDTask(void const * argument);
void INFOTask(void const * argument);
void OneNET_Task(void const * argument);
void Timer01_Callback(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of LED_FLAG_Sem */
  osSemaphoreDef(LED_FLAG_Sem);
  LED_FLAG_SemHandle = osSemaphoreCreate(osSemaphore(LED_FLAG_Sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Timer01_Callback);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, float);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* definition and creation of myQueue02 */
  osMessageQDef(myQueue02, 16, _Bool);
  myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of BH1750_Task */
  osThreadDef(BH1750_Task, BH1750Task, osPriorityNormal, 0, 128);
  BH1750_TaskHandle = osThreadCreate(osThread(BH1750_Task), NULL);

  /* definition and creation of LED_TASK */
  osThreadDef(LED_TASK, LEDTask, osPriorityNormal, 0, 128);
  LED_TASKHandle = osThreadCreate(osThread(LED_TASK), NULL);

  /* definition and creation of INFO_Task */
  osThreadDef(INFO_Task, INFOTask, osPriorityNormal, 0, 128);
  INFO_TaskHandle = osThreadCreate(osThread(INFO_Task), NULL);

  /* definition and creation of OneNETTask */
  osThreadDef(OneNETTask, OneNET_Task, osPriorityNormal, 0, 128);
  OneNETTaskHandle = osThreadCreate(osThread(OneNETTask), NULL);

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
	int dis_data = 0;
	uint8_t DATA_BUF[8] = { 0 };
	
	Init_BH1750();
	HAL_Delay(200);
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
		dis_data=(dis_data<<8)+ (DATA_BUF[1]);														//合成数据
		LIGH=(float)dis_data/1.2;
		LIGH_OneNET_buf = LIGH;																						//将光照强度数据放入onenet的数据缓冲区中
		xQueueSendToFront(myQueue01Handle,&LIGH,portMAX_DELAY);
		
		osDelay(1);
  }
  /* USER CODE END BH1750Task */
}

/* USER CODE BEGIN Header_LEDTask */
/**
* @brief Function implementing the LED_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask */
void LEDTask(void const * argument)
{
  /* USER CODE BEGIN LEDTask */
	float LIGH = 0;
	volatile uint32_t ul; 																							//volatile用来避免被优化掉
	_Bool LED_FLAG;
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(myQueue01Handle,&LIGH,portMAX_DELAY);
		if(LIGH<20)
		{
			HAL_GPIO_WritePin(GPIOC, D4_Pin|D5_Pin, GPIO_PIN_RESET);				//开灯
			LED_FLAG =1;																										//LED开启标志位置1
			if(xSemaphoreGive(LED_FLAG_SemHandle) == pdTRUE)								//如果开启LED(光照强度小于20)，则释放信号量，LED_FLAG_Sem加1
			{
				xQueueSendToFront(myQueue02Handle,&LED_FLAG,portMAX_DELAY);
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, D4_Pin|D5_Pin, GPIO_PIN_SET);
			LED_FLAG =0;
			if(xSemaphoreTake(LED_FLAG_SemHandle,0) == pdTRUE)							//如果关闭LED(光照强度大于20)，则获取信号量，LED_FLAG_Sem减1，如果信号量无余量，则立即返回
			{
				xQueueSendToFront(myQueue02Handle,&LED_FLAG,portMAX_DELAY);
			}
		}
		LEDS_OneNET_buf = LED_FLAG;
    osDelay(1);
  }
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_INFOTask */
/**
* @brief Function implementing the INFO_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INFOTask */
void INFOTask(void const * argument)
{
  /* USER CODE BEGIN INFOTask */
	_Bool LED_FLAG;
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(myQueue02Handle,&LED_FLAG,portMAX_DELAY);						//接收消息队列中LED开启标志位的最新状态
		if(LED_FLAG)
		{
			printf("\r\nLED ON\r\n");
			LED_FLAG = NULL;
		}else{
			printf("\r\nLED OFF\r\n");
			LED_FLAG = NULL;
		}
    osDelay(1);
  }
  /* USER CODE END INFOTask */
}

/* USER CODE BEGIN Header_OneNET_Task */
/**
* @brief Function implementing the OneNETTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OneNET_Task */
void OneNET_Task(void const * argument)
{
  /* USER CODE BEGIN OneNET_Task */
	unsigned short timeCount = 0;	
	unsigned char *dataPtr = NULL;
	
	ESP8266_Init();
	while(OneNet_DevLink());
	printf("连接ONENET成功\t[OK]\r\n");
	
  /* Infinite loop */
  for(;;)
  {
		if(++timeCount >= 100)				//2500ms / 25 = 100 发送间隔2500ms
		{
			OneNet_SendData();					//打包数据包发送到ONENET云平台，一次只能打包一个数据流发送，若要发得更快就调整发送时间间隔
			timeCount = 0;							//时间间隔计时清0
			ESP8266_Clear();						//数据包清空
		}
		dataPtr = ESP8266_GetIPD(3);	//检测平台是否有响应
		if(dataPtr != NULL)
		OneNet_RevPro(dataPtr);
		
    osDelay(1);
  }
  /* USER CODE END OneNET_Task */
}

/* Timer01_Callback function */
void Timer01_Callback(void const * argument)
{
  /* USER CODE BEGIN Timer01_Callback */

  /* USER CODE END Timer01_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
