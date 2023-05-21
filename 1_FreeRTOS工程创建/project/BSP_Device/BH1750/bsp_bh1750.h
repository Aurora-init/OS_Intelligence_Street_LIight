/*_BSP_BH1750_H_*/
#ifndef _BSP_BH1750_H_
#define _BSP_BH1750_H_
#include "main.h"
#include <stdio.h>
#include "i2c.h"

#define BH1750_ADDRESS 0x46

void Init_BH1750(void);
uint32_t I2C_BH1750_Opecode_Write(uint8_t* pData, uint16_t size);
uint32_t I2C_BH1750_Data_Read(uint8_t* pData, uint16_t size);
#endif

