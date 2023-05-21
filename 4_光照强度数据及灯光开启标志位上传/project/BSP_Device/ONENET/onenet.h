#ifndef _ONENET_H_
#define _ONENET_H_

#include "ESP/bsp_esp01s.h"				//网络设备
#include "MQTT/mqttkit.h"					//SDK文件
#include "COMMON/bsp_headfile.h"
#include "COMMON/bsp_common.h"	

#define PROID			"535146"				//产品ID
#define DEVID			"970751245"			//设备ID
#define AUTH_INFO	"123" 					//鉴权信息

_Bool OneNet_DevLink(void);

void OneNet_Subscribe(const char *topics[], unsigned char topic_cnt);

void OneNet_Publish(const char *topic, const char *msg);

void OneNet_RevPro(unsigned char *cmd);

void OneNet_SendData(void);

#endif
