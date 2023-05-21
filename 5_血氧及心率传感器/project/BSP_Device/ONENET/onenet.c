#include "ONENET/onenet.h"

char text[16];
uint8_t flag_change = 0;

/*
************************************************************
*	�������ƣ�	OneNet_DevLink
*
*	�������ܣ�	��onenet��������
*
*	��ڲ�����	��
*
*	���ز�����	1-�ɹ�	0-ʧ��
*
*	˵����			��onenetƽ̨��������
************************************************************
*/
_Bool OneNet_DevLink(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};						//Э���
	unsigned char *dataPtr;
	_Bool status = 1;
	
	UsartPrintf(&huart1, "OneNet_DevLink\r\n""PROID: %s,	AUIF: %s,	DEVID:%s\r\n" , PROID, AUTH_INFO, DEVID);
	if(MQTT_PacketConnect(PROID, AUTH_INFO, DEVID, 256, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);				//�ϴ�ƽ̨	
		UsartPrintf(&huart1, "�����ϴ����\r\n�ȴ���Ӧ\r\n");
		dataPtr = ESP8266_GetIPD(250);															//�ȴ�ƽ̨��Ӧ										
		if(dataPtr != NULL)
		{
			UsartPrintf(&huart1, "ƽ̨����Ӧ\r\n");
			if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
			{
				switch(MQTT_UnPacketConnectAck(dataPtr))
				{
					case 0:	UsartPrintf(&huart1, "Tips:	���ӳɹ�\r\n");status = 0;								break;
					case 1:	UsartPrintf(&huart1, "WARN:	����ʧ�ܣ�Э�����\r\n");									break;
					case 2:	UsartPrintf(&huart1, "WARN:	����ʧ�ܣ��Ƿ���clientid\r\n");						break;
					case 3:	UsartPrintf(&huart1, "WARN:	����ʧ�ܣ�������ʧ��\r\n");								break;
					case 4:	UsartPrintf(&huart1, "WARN:	����ʧ�ܣ��û������������\r\n");					break;
					case 5:	UsartPrintf(&huart1, "WARN:	����ʧ�ܣ��Ƿ�����(����token�Ƿ�)\r\n");	break;
					default:UsartPrintf(&huart1, "ERR:	����ʧ�ܣ�δ֪����\r\n");									break;
				}
			}
		}
		MQTT_DeleteBuffer(&mqttPacket);															//ɾ��				
	}
	else
	{	
		UsartPrintf(&huart1, "ƽ̨����Ӧ\r\n");
		UsartPrintf(&huart1, "WARN:	MQTT_PacketConnect Failed\r\n");
	}
	return status;
}

/*
************************************************************
*	�������ƣ�	OneNet_Subscribe
*
*	�������ܣ�	����
*
*	��ڲ�����	topics�����ĵ�topic
*							topic_cnt��topic����
*
*	���ز�����	SEND_TYPE_OK-�ɹ�	SEND_TYPE_SUBSCRIBE-��Ҫ�ط�
*
*	˵����			
************************************************************
*/
void OneNet_Subscribe(const char *topics[], unsigned char topic_cnt)
{
	unsigned char i = 0;
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};			//Э���				
	for(; i < topic_cnt; i++)
	{
		UsartPrintf(&huart1, "Subscribe Topic: %s\r\n", topics[i]);
	}
	if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL0, topics, topic_cnt, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);	//��ƽ̨���Ͷ�������				
		MQTT_DeleteBuffer(&mqttPacket);												//ɾ��
	}
}

/*
************************************************************
*	�������ƣ�	OneNet_Publish
*
*	�������ܣ�	������Ϣ
*
*	��ڲ�����	topic������������
*							msg����Ϣ����
*
*	���ز�����	SEND_TYPE_OK-�ɹ�	SEND_TYPE_PUBLISH-��Ҫ����
*
*	˵����			
************************************************************
*/
void OneNet_Publish(const char *topic, const char *msg)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};			//Э���				
	UsartPrintf(&huart1, "Publish Topic: %s, Msg: %s\r\n", topic, msg);
	if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic, msg, strlen(msg), MQTT_QOS_LEVEL0, 0, 1, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);	//��ƽ̨���Ͷ�������					
		MQTT_DeleteBuffer(&mqttPacket);												//ɾ��
	}
}

/*
************************************************************
*	�������ƣ�	OneNet_FillBuf
*
*	�������ܣ�	���������ϢΪJSON��ʽ
*
*	��ڲ�����	buf��onenet���ݻ�����
*
*	���ز�����	���ݻ������������ݳ���
*
*	˵����			���������Ϣ
************************************************************
*/
unsigned char OneNet_FillBuf(char *buf)
{
	memset(text, 0, sizeof(text));			//��ʼ��������BUFF
	if(flag_change == 1)								//����ǿ��LIGH������
	{
		printf("\r\n�����������ǿ��: %f\r\n",LIGH_OneNET_buf);
		strcpy(buf, "{");
		memset(text, 0, sizeof(text));
		sprintf(text, "\"LIGH\":%f",LIGH_OneNET_buf);
		strcat(buf, text);
		strcat(buf, "}");
		flag_change = 2;
	}
	if(flag_change == 0)								//�ƹ�״̬LEDS������
	{
		printf("\r\n��������ƹ�״̬: %d\r\n",LEDS_OneNET_buf);
		strcpy(buf, "{");
		memset(text, 0, sizeof(text));
		sprintf(text, "\"LEDS\":%d",LEDS_OneNET_buf);
		strcat(buf, text);
		strcat(buf, "}");
		flag_change = 1;
	}
	if(flag_change == 2)								//����������
	{
		flag_change = 0;
	}
	return strlen(buf);
}

/*
************************************************************
*	�������ƣ�	OneNet_SendData
*
*	�������ܣ�	�ϴ����ݵ�ƽ̨
*
*	��ڲ�����	type���������ݵĸ�ʽ
*
*	���ز�����	��
*
*	˵����			
************************************************************
*/
void OneNet_SendData(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};														//Э���						
	char buf[128];
	short body_len = 0, i = 0;
	
	memset(buf, 0, sizeof(buf));
	body_len = OneNet_FillBuf(buf);																								//��ȡ��ǰ��Ҫ���͵����������ܳ���
	if(body_len)
	{
		if(MQTT_PacketSaveData(DEVID, body_len, NULL, 3, &mqttPacket) == 0)					//���				
		{
			for(; i < body_len; i++)
			{
				mqttPacket._data[mqttPacket._len++] = buf[i];
			}
			ESP8266_SendData(mqttPacket._data, mqttPacket._len);											//�ϴ����ݵ�ƽ̨
			UsartPrintf(&huart1, "Send %d Bytes\r\n", mqttPacket._len);
			MQTT_DeleteBuffer(&mqttPacket);																						//ɾ��
		}
		else
			UsartPrintf(&huart1, "WARN:	MQTT_NewBuffer Failed\r\n");
	}
}

/*
************************************************************
*	�������ƣ�	OneNet_RevPro
*
*	�������ܣ�	ƽ̨�������ݼ��
*
*	��ڲ�����	dataPtr��ƽ̨���ص�����
*
*	���ز�����	��
*
*	˵����			
************************************************************
*/
void OneNet_RevPro(unsigned char *cmd)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};														//Э���
	char *req_payload = NULL;
	char *cmdid_topic = NULL;
	
	unsigned short req_len = 0;
	unsigned char type = 0;
	short result = 0;
	
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
		//�����·�
		case MQTT_PKT_CMD:		
			
			result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload, &req_len);		//���topic����Ϣ��
			if(result == 0)
			{
				UsartPrintf(&huart1, "cmdid: %s, req: %s, req_len: %d\r\n", cmdid_topic, req_payload, req_len);
				if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)		//����ظ����
				{
					UsartPrintf(&huart1, "Tips:	Send CmdResp\r\n");
					ESP8266_SendData(mqttPacket._data, mqttPacket._len);								//�ظ�����
					MQTT_DeleteBuffer(&mqttPacket);																			//ɾ��
				}
			}
		break;
			
		case MQTT_PKT_PUBACK:																											//����Publish��Ϣ��ƽ̨�ظ���Ack
		
			if(MQTT_UnPacketPublishAck(cmd) == 0)
//				UsartPrintf(&huart1, "Tips:	MQTT Publish Send OK\r\n");
			
		break;
		
		default:
			result = -1;
		break;
	}
	
	ESP8266_Clear();//��ջ���
	
	if(result == -1)
		return;

	if(strstr((char *)req_payload, "OPEN"))//����"OPEN"
	{
			UsartPrintf(&huart1, "Blue_Led_ON\r\n");
	}
	else if(strstr((char *)req_payload, "CLOED"))//����"CLOED"
	{
			UsartPrintf(&huart1, "Blue_Led_OFF\r\n");
	}
	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
	}
}
