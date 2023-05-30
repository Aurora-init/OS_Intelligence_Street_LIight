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
#include "BH1750/bsp_bh1750.h"
#include "ESP/bsp_esp01s.h"
#include "ONENET/onenet.h"
#include "MQTT/MqttKit.h"
#include "MAX30102/bsp_max30102.h"
#include "MAX30102/bsp_max30102_fir.h"
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
RTC_DateTypeDef GetData;  	//��ȡ���ڽṹ��
RTC_TimeTypeDef GetTime;   	//��ȡʱ��ṹ��

uint8_t FeedDog_Flag = 0;

float LIGH_OneNET_buf;
uint8_t LEDS_OneNET_buf;

static volatile uint8_t flagTimer01Run = 0;

uint8_t max30102_int_flag = 0; // �жϱ�־
float ppg_data_cache_RED[CACHE_NUMS] = {0}; // ������
float ppg_data_cache_IR[CACHE_NUMS] = {0};  // ������
uint16_t cache_counter = 0; // ���������

uint16_t HeartRateold = 0;

/* USER CODE END Variables */
osThreadId BH1750_TaskHandle;
osThreadId LED_TASKHandle;
osThreadId INFO_TaskHandle;
osThreadId OneNETTaskHandle;
osThreadId MAX30102_TaskHandle;
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
void MAX30102Task(void const * argument);
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
	osTimerStart(myTimer01Handle,20000);							//10000����ص������ص�����
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

  /* definition and creation of MAX30102_Task */
  osThreadDef(MAX30102_Task, MAX30102Task, osPriorityAboveNormal, 0, 128);
  MAX30102_TaskHandle = osThreadCreate(osThread(MAX30102_Task), NULL);

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
			return;
		}
		opecode = 0x10;
		if ( I2C_BH1750_Opecode_Write(&opecode, 1) != HAL_OK){
			printf("I2C_BH1750_Opecode_Write: %2x Error\n", opecode);
			return;
		}
		HAL_Delay(200);	
		if ( I2C_BH1750_Data_Read(DATA_BUF, 2) != HAL_OK){
			printf("I2C_BH1750_Data_Read:  Error\n");
			return;
		}
		dis_data =(DATA_BUF[0]);
		dis_data=(dis_data<<8)+ (DATA_BUF[1]);														//�ϳ�����
		LIGH=(float)dis_data/1.2;
		LIGH_OneNET_buf = LIGH;																						//������ǿ�����ݷ���onenet�����ݻ�������
		xQueueSendToFront(myQueue01Handle,&LIGH,portMAX_DELAY);						//myQueue01��Ϣ��������BH1750Task��LEDTask�������ݽ���
		
		osDelay(2000);
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
	volatile uint32_t ul; 																							//volatile�������ⱻ�Ż���
	_Bool LED_FLAG;
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(myQueue01Handle,&LIGH,portMAX_DELAY);
		if(LIGH<20)
		{
			HAL_GPIO_WritePin(GPIOC, D4_Pin|D5_Pin, GPIO_PIN_RESET);				//����
			LED_FLAG =1;																										//LED������־λ��1
			if(xSemaphoreGive(LED_FLAG_SemHandle) == pdTRUE)								//�������LED(����ǿ��С��20)�����ͷ��ź�����LED_FLAG_Sem��1
			{
				xQueueSendToFront(myQueue02Handle,&LED_FLAG,portMAX_DELAY);
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, D4_Pin|D5_Pin, GPIO_PIN_SET);
			LED_FLAG =0;
			if(xSemaphoreTake(LED_FLAG_SemHandle,0) == pdTRUE)							//����ر�LED(����ǿ�ȴ���20)�����ȡ�ź�����LED_FLAG_Sem��1������ź���������������������
			{
				xQueueSendToFront(myQueue02Handle,&LED_FLAG,portMAX_DELAY);
			}
		}
		LEDS_OneNET_buf = LED_FLAG;
    osDelay(1000);
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
		xQueueReceive(myQueue02Handle,&LED_FLAG,portMAX_DELAY);						//������Ϣ������LED������־λ������״̬
		if(LED_FLAG)
		{
			printf("\r\nLED ON\r\n");
			LED_FLAG = NULL;
		}else{
			printf("\r\nLED OFF\r\n");
			LED_FLAG = NULL;
		}
    osDelay(1000);
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
	printf("����ONENET�ɹ�\t[OK]\r\n");
	
  /* Infinite loop */
  for(;;)
  {
		if(++timeCount >= 100)				//2500ms / 25 = 100 ���ͼ��2500ms
		{
			OneNet_SendData();					//������ݰ����͵�ONENET��ƽ̨��һ��ֻ�ܴ��һ�����������ͣ���Ҫ���ø���͵�������ʱ����
			timeCount = 0;							//ʱ������ʱ��0
			ESP8266_Clear();						//���ݰ����
		}
		dataPtr = ESP8266_GetIPD(3);	//���ƽ̨�Ƿ�����Ӧ
		if(dataPtr != NULL)
		OneNet_RevPro(dataPtr);
		
    osDelay(100);
  }
  /* USER CODE END OneNET_Task */
}

/* USER CODE BEGIN Header_MAX30102Task */
/**
* @brief Function implementing the MAX30102_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MAX30102Task */
void MAX30102Task(void const * argument)
{
  /* USER CODE BEGIN MAX30102Task */
	uint16_t HeartRate = 0;
	float SpO2 = 0;
	float max30102_data[2], fir_output[2];
	
	max30102_init();
  max30102_fir_init();
	
  printf("Max30102 Init\tOK\r\n");
	
  /* Infinite loop */
  for(;;)
  {
		if(MAX30102_Get_DATA(&HeartRate,&SpO2,max30102_data,fir_output) == MAX30102_DATA_OK)
		{
			
			printf("\r\n���ʣ�%d��/min\r\n"	, HeartRate); //��ǰ�����ʴ�ӡ����
			if(HeartRate-HeartRateold>=50)
			{
				printf("\r\nѹ���ô�\r\n"); //��ǰ��ѹ��
			}
			else if(HeartRate-HeartRateold<50&&HeartRate-HeartRateold>30)
			{
				printf("\r\nѹ������\r\n"); //��ǰ��ѹ��
			}
			else
			{
				printf("\r\n��ȫûѹ��\r\n"); //��ǰ��ѹ��
			}
			HeartRateold = HeartRate;
			printf("Ѫ����%.2f%%\r\n"		, SpO2);
		}
    osDelay(1);
  }
  /* USER CODE END MAX30102Task */
}

/* Timer01_Callback function */
void Timer01_Callback(void const * argument)
{
  /* USER CODE BEGIN Timer01_Callback */
	HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);

	if (FeedDog_Flag == FeedDog)
	{
		FeedDog_Flag = NoFeedDog;
		
		printf("\r\n%02d/%02d/%02d\r\n",2000 + GetData.Year, GetData.Month, GetData.Date);

		printf("%02d:%02d:%02d\r\n",GetTime.Hours, GetTime.Minutes, GetTime.Seconds);
	}
  /* USER CODE END Timer01_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
