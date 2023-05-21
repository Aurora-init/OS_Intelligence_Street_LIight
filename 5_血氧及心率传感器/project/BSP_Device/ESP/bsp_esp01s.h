#ifndef __BSP_ESP01S_H_
#define __BSP_ESP01S_H_

#include "COMMON/bsp_headfile.h"
#include "COMMON/bsp_common.h"

#define REV_OK											0																												//������ɱ�־
#define REV_WAIT										1																												//����δ��ɱ�־
#define ESP8266_WIFI_INFO						"AT+CWJAP=\"Ace2\",\"123780807\"\r\n"										//wifi_ssid��wifi_password
#define ESP8266_ONENET_INFO        	"AT+CIPSTART=\"TCP\",\"183.230.40.39\",6002\r\n"				//onenet��ѵ��豸

typedef struct
{
	uint8_t *IPD_buff;																																				//������·�������
	uint16_t len; 		 																																				//�������ݳ���
}IPD_RX;

extern unsigned char USART_TX_BUF[400]; 																										
extern unsigned char esp8266_buf[128];
extern unsigned short esp8266_cnt;
extern unsigned short esp8266_cntPre;

void Usart_SendString(UART_HandleTypeDef *USARTx, unsigned char *str, unsigned short len);

void UsartPrintf(UART_HandleTypeDef *USARTx, char *fmt,...);

void ESP8266_Clear(void);

_Bool ESP8266_WaitRecive(void);

_Bool ESP8266_SendCmd(char *cmd, char *res);

void ESP8266_SendData(unsigned char *data, unsigned short len);

unsigned char *ESP8266_GetIPD(unsigned short timeOut);

void ESP8266_Init(void);

#endif

