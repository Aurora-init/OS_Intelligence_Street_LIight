#include "ESP/bsp_esp01s.h"

unsigned char USART_TX_BUF[400]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节
unsigned char esp8266_buf[128];
unsigned short esp8266_cnt = 0;
unsigned short esp8266_cntPre = 0;

/*
************************************************************
*	函数名称：	Usart_SendString
*
*	函数功能：	串口数据发送
*
*	入口参数：	USARTx：	串口组
*							str：			要发送的数据
*							len：			数据长度
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void Usart_SendString(UART_HandleTypeDef *USARTx, unsigned char *str, unsigned short len)
{
	uint16_t j;
	for(j=0;j<len;j++)
	{
		while((USART2->SR&0X40)==0); 
		USART2->DR=*str++;  
	}
}

/*
************************************************************
*	函数名称：	UsartPrintf
*
*	函数功能：	格式化打印
*
*	入口参数：	USARTx：		串口组
*							fmt：				不定长参
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void UsartPrintf(UART_HandleTypeDef *USARTx, char *fmt,...)
{
	uint16_t i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART_TX_BUF);		
	for(j=0;j<i;j++)
	{
		while((USART1->SR&0X40)==0);					
		USART1->DR=USART_TX_BUF[j];  
	}
}

/*
************************************************************
*	函数名称：	ESP8266_Clear
*
*	函数功能：	清空缓存
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void ESP8266_Clear(void)
{
	memset(esp8266_buf, 0, sizeof(esp8266_buf));
	esp8266_cnt = 0;
}

/*
************************************************************
*	函数名称：	ESP8266_WaitRecive
*
*	函数功能：	等待接收完成
*
*	入口参数：	无
*
*	返回参数：	REV_OK-接收完成		REV_WAIT-接收超时未完成
*
*	说明：			循环调用检测是否接收完成
************************************************************
*/
_Bool ESP8266_WaitRecive(void)
{
	if(esp8266_cnt == 0) 										//如果接收计数为0 则说明没有处于接收数据中，所以直接跳出，结束函数
	{
		return REV_WAIT;
	}
	else if(esp8266_cnt == esp8266_cntPre)	//如果上一次的值和这次相同，则说明接收完毕
	{
		esp8266_cnt = 0;											//清0接收计数
		return REV_OK;												//返回接收完成标志
	}
	esp8266_cntPre = esp8266_cnt;						//置为相同
	return REV_WAIT;												//返回接收未完成标志
}

/*
************************************************************
*	函数名称：	ESP8266_SendCmd
*
*	函数功能：	发送命令
*
*	入口参数：	cmd：		命令
*							res：		需要检查的返回指令
*
*	返回参数：	0-成功	1-失败
*
*	说明：			
************************************************************
*/
_Bool ESP8266_SendCmd(char *cmd, char *res)
{
	unsigned char timeOut = 200;
	Usart_SendString(&huart2, (unsigned char *)cmd, strlen((const char *)cmd));
	while(timeOut--)
	{
		if(ESP8266_WaitRecive() == REV_OK)											//如果收到数据
		{
			if(strstr((const char *)esp8266_buf, res) != NULL)		//如果检索到关键词
			{
				ESP8266_Clear();																		//清空缓存
				return 0;
			}
		}
		HAL_Delay(10);
	}
	return 1;
}

/*
************************************************************
*	函数名称：	ESP8266_SendData
*
*	函数功能：	发送数据
*
*	入口参数：	data：		数据
*							len：			长度
*
*	返回参数：	无
*
*	说明：			
************************************************************
*/
void ESP8266_SendData(unsigned char *data, unsigned short len)
{
	char cmdBuf[32];															//命令缓冲区
	ESP8266_Clear();															//清空接收缓存
	sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);		//发送命令
	if(!ESP8266_SendCmd(cmdBuf, ">"))							//收到‘>’时可以发送数据
	{
		Usart_SendString(&huart2, data, len);				//发送设备连接请求数据
	}
}

/*
************************************************************
*	函数名称：	ESP8266_GetIPD
*
*	函数功能：	获取平台返回的数据
*
*	入口参数：	等待的时间(乘以10ms)
*
*	返回参数：	平台返回的原始数据
*
*	说明：			不同网络设备返回的格式不同，需要去调试，如ESP8266的返回格式为	"+IPD,x:yyy"	x代表数据长度，yyy是数据内容		
************************************************************
*/
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
	char *ptrIPD = NULL;
	do
	{
		if(ESP8266_WaitRecive() == REV_OK)							//如果接收完成							
		{
			ptrIPD = strstr((char *)esp8266_buf, "IPD,");	//搜索“IPD”头				
			if(ptrIPD == NULL)														//如果没找到，可能是IPD头的延迟，还是需要等待一会，但不会超过设定的时间					
			{	
			}else{
				ptrIPD = strchr(ptrIPD, ':');								//找到':'
				if(ptrIPD != NULL)
				{
					ptrIPD++;
					return (unsigned char *)(ptrIPD);
				}else{
					return NULL;
				}
			}
		}
		HAL_Delay(5);																		//延时等待															
	}while(timeOut--);
	return NULL;																			//超时还未找到，返回空指针
}

/*
************************************************************
*	函数名称：	ESP8266_Init
*
*	函数功能：	AT指令的方式初始化ESP01S
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：			
************************************************************
*/
void ESP8266_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	ESP8266_Clear();
	
//	UsartPrintf(&huart1, "\r\n0. AT\r\n");
	while(ESP8266_SendCmd("AT\r\n", "OK"))
	HAL_Delay(200);
	
	UsartPrintf(&huart1, "\r\n1. RST\r\n");
	ESP8266_SendCmd("AT+RST\r\n", "");
	HAL_Delay(200);
	
	ESP8266_SendCmd("AT+CIPCLOSE\r\n", "");
	HAL_Delay(200);
	
	UsartPrintf(&huart1, "\r\n2. CWMODE\r\n");
	while(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK"))
	HAL_Delay(200);
	
	UsartPrintf(&huart1, "\r\n3. AT+CWDHCP\r\n");
	while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))
	HAL_Delay(200);
	
	UsartPrintf(&huart1, "\r\n4. CWJAP\r\n");
	while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
	HAL_Delay(200);
	
	UsartPrintf(&huart1, "\r\n5. CIPSTART\r\n");
	while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
	HAL_Delay(200);
	
	UsartPrintf(&huart1, "\r\n6. ESP8266 Init OK\r\n");
}
