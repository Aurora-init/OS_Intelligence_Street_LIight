#include "ONENET/onenet.h"

char text[16];
uint8_t flag_change = 0;

/*
************************************************************
*	函数名称：	OneNet_DevLink
*
*	函数功能：	与onenet创建连接
*
*	入口参数：	无
*
*	返回参数：	1-成功	0-失败
*
*	说明：			与onenet平台建立连接
************************************************************
*/
_Bool OneNet_DevLink(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};						//协议包
	unsigned char *dataPtr;
	_Bool status = 1;
	
	UsartPrintf(&huart1, "OneNet_DevLink\r\n""PROID: %s,	AUIF: %s,	DEVID:%s\r\n" , PROID, AUTH_INFO, DEVID);
	if(MQTT_PacketConnect(PROID, AUTH_INFO, DEVID, 256, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);				//上传平台	
		UsartPrintf(&huart1, "连接上传完成\r\n等待响应\r\n");
		dataPtr = ESP8266_GetIPD(250);															//等待平台响应										
		if(dataPtr != NULL)
		{
			UsartPrintf(&huart1, "平台已响应\r\n");
			if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
			{
				switch(MQTT_UnPacketConnectAck(dataPtr))
				{
					case 0:	UsartPrintf(&huart1, "Tips:	连接成功\r\n");status = 0;								break;
					case 1:	UsartPrintf(&huart1, "WARN:	连接失败：协议错误\r\n");									break;
					case 2:	UsartPrintf(&huart1, "WARN:	连接失败：非法的clientid\r\n");						break;
					case 3:	UsartPrintf(&huart1, "WARN:	连接失败：服务器失败\r\n");								break;
					case 4:	UsartPrintf(&huart1, "WARN:	连接失败：用户名或密码错误\r\n");					break;
					case 5:	UsartPrintf(&huart1, "WARN:	连接失败：非法链接(比如token非法)\r\n");	break;
					default:UsartPrintf(&huart1, "ERR:	连接失败：未知错误\r\n");									break;
				}
			}
		}
		MQTT_DeleteBuffer(&mqttPacket);															//删包				
	}
	else
	{	
		UsartPrintf(&huart1, "平台无响应\r\n");
		UsartPrintf(&huart1, "WARN:	MQTT_PacketConnect Failed\r\n");
	}
	return status;
}

/*
************************************************************
*	函数名称：	OneNet_Subscribe
*
*	函数功能：	订阅
*
*	入口参数：	topics：订阅的topic
*							topic_cnt：topic个数
*
*	返回参数：	SEND_TYPE_OK-成功	SEND_TYPE_SUBSCRIBE-需要重发
*
*	说明：			
************************************************************
*/
void OneNet_Subscribe(const char *topics[], unsigned char topic_cnt)
{
	unsigned char i = 0;
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};			//协议包				
	for(; i < topic_cnt; i++)
	{
		UsartPrintf(&huart1, "Subscribe Topic: %s\r\n", topics[i]);
	}
	if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL0, topics, topic_cnt, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);	//向平台发送订阅请求				
		MQTT_DeleteBuffer(&mqttPacket);												//删包
	}
}

/*
************************************************************
*	函数名称：	OneNet_Publish
*
*	函数功能：	发布消息
*
*	入口参数：	topic：发布的主题
*							msg：消息内容
*
*	返回参数：	SEND_TYPE_OK-成功	SEND_TYPE_PUBLISH-需要重送
*
*	说明：			
************************************************************
*/
void OneNet_Publish(const char *topic, const char *msg)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};			//协议包				
	UsartPrintf(&huart1, "Publish Topic: %s, Msg: %s\r\n", topic, msg);
	if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic, msg, strlen(msg), MQTT_QOS_LEVEL0, 0, 1, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);	//向平台发送订阅请求					
		MQTT_DeleteBuffer(&mqttPacket);												//删包
	}
}

/*
************************************************************
*	函数名称：	OneNet_FillBuf
*
*	函数功能：	打包数据信息为JSON格式
*
*	入口参数：	buf：onenet数据缓冲区
*
*	返回参数：	数据缓冲区所存数据长度
*
*	说明：			打包数据信息
************************************************************
*/
unsigned char OneNet_FillBuf(char *buf)
{
	memset(text, 0, sizeof(text));			//初始化数据流BUFF
	if(flag_change == 1)								//光照强度LIGH数据流
	{
		printf("\r\n本次输出光照强度: %f\r\n",LIGH_OneNET_buf);
		strcpy(buf, "{");
		memset(text, 0, sizeof(text));
		sprintf(text, "\"LIGH\":%f",LIGH_OneNET_buf);
		strcat(buf, text);
		strcat(buf, "}");
		flag_change = 2;
	}
	if(flag_change == 0)								//灯光状态LEDS数据流
	{
		printf("\r\n本次输出灯光状态: %d\r\n",LEDS_OneNET_buf);
		strcpy(buf, "{");
		memset(text, 0, sizeof(text));
		sprintf(text, "\"LEDS\":%d",LEDS_OneNET_buf);
		strcat(buf, text);
		strcat(buf, "}");
		flag_change = 1;
	}
	if(flag_change == 2)								//数据流回溯
	{
		flag_change = 0;
	}
	return strlen(buf);
}

/*
************************************************************
*	函数名称：	OneNet_SendData
*
*	函数功能：	上传数据到平台
*
*	入口参数：	type：发送数据的格式
*
*	返回参数：	无
*
*	说明：			
************************************************************
*/
void OneNet_SendData(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};														//协议包						
	char buf[128];
	short body_len = 0, i = 0;
	
	memset(buf, 0, sizeof(buf));
	body_len = OneNet_FillBuf(buf);																								//获取当前需要发送的数据流的总长度
	if(body_len)
	{
		if(MQTT_PacketSaveData(DEVID, body_len, NULL, 3, &mqttPacket) == 0)					//封包				
		{
			for(; i < body_len; i++)
			{
				mqttPacket._data[mqttPacket._len++] = buf[i];
			}
			ESP8266_SendData(mqttPacket._data, mqttPacket._len);											//上传数据到平台
			UsartPrintf(&huart1, "Send %d Bytes\r\n", mqttPacket._len);
			MQTT_DeleteBuffer(&mqttPacket);																						//删包
		}
		else
			UsartPrintf(&huart1, "WARN:	MQTT_NewBuffer Failed\r\n");
	}
}

/*
************************************************************
*	函数名称：	OneNet_RevPro
*
*	函数功能：	平台返回数据检测
*
*	入口参数：	dataPtr：平台返回的数据
*
*	返回参数：	无
*
*	说明：			
************************************************************
*/
void OneNet_RevPro(unsigned char *cmd)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};														//协议包
	char *req_payload = NULL;
	char *cmdid_topic = NULL;
	
	unsigned short req_len = 0;
	unsigned char type = 0;
	short result = 0;
	
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
		//命令下发
		case MQTT_PKT_CMD:		
			
			result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload, &req_len);		//解出topic和消息体
			if(result == 0)
			{
				UsartPrintf(&huart1, "cmdid: %s, req: %s, req_len: %d\r\n", cmdid_topic, req_payload, req_len);
				if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)		//命令回复组包
				{
					UsartPrintf(&huart1, "Tips:	Send CmdResp\r\n");
					ESP8266_SendData(mqttPacket._data, mqttPacket._len);								//回复命令
					MQTT_DeleteBuffer(&mqttPacket);																			//删包
				}
			}
		break;
			
		case MQTT_PKT_PUBACK:																											//发送Publish消息，平台回复的Ack
		
			if(MQTT_UnPacketPublishAck(cmd) == 0)
//				UsartPrintf(&huart1, "Tips:	MQTT Publish Send OK\r\n");
			
		break;
		
		default:
			result = -1;
		break;
	}
	
	ESP8266_Clear();//清空缓存
	
	if(result == -1)
		return;

	if(strstr((char *)req_payload, "OPEN"))//搜索"OPEN"
	{
			UsartPrintf(&huart1, "Blue_Led_ON\r\n");
	}
	else if(strstr((char *)req_payload, "CLOED"))//搜索"CLOED"
	{
			UsartPrintf(&huart1, "Blue_Led_OFF\r\n");
	}
	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
	}
}
