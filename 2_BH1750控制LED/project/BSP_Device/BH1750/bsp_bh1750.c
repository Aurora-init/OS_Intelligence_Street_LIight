/*_BSP_BH1750_C_*/

#include "BH1750/bsp_bh1750.h"

void Init_BH1750(void) {
	uint8_t opecode = 0x01;
	if( I2C_BH1750_Opecode_Write(&opecode, 1) == HAL_OK ){
		printf("Init_BH1750 Ok\n");
	}else {
		printf("Init_BH1750 Error\n");
	}
}
uint32_t I2C_BH1750_Opecode_Write(uint8_t* pData, uint16_t size) {
	HAL_StatusTypeDef status = HAL_OK;	
	status = HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDRESS,pData, size, 1);	
	return status;	
}
uint32_t I2C_BH1750_Data_Read(uint8_t* pData, uint16_t size){
	HAL_StatusTypeDef status = HAL_OK;	
	status = HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDRESS+1,pData, size, 1);	
	return status;
}

