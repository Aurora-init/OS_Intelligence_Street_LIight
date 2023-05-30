/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : max30102.c
 * @brief          : 血氧传感器
 ******************************************************************************
 * @attention
 * 1.要宏定义 ARM_MATH_CM4,__FPU_PRESENT
 * 2.打开DSP
 * 3.main函数定义如下全局变量
 *	uint8_t max30102_int_flag = 0; // 中断标志
 *	float ppg_data_cache_RED[CACHE_NUMS] = {0}; // 缓存区
 *	float ppg_data_cache_IR[CACHE_NUMS] = {0};  // 缓存区
 *	uint16_t cache_counter = 0; // 缓存计数器
 * 
 ******************************************************************************
 */
/* USER CODE END Header */
#include "MAX30102/bsp_max30102.h"
#include "MAX30102/bsp_max30102_fir.h"
 
extern uint8_t max30102_int_flag;
extern float ppg_data_cache_RED[CACHE_NUMS]; // 缓存区
extern float ppg_data_cache_IR[CACHE_NUMS];  // 缓存区
extern uint16_t cache_counter;

/**
 * @brief IIC 写入
 * @retval None
 */
void max30102_i2c_write(uint8_t reg_adder, uint8_t data)
{
	uint8_t transmit_data[2];
	transmit_data[0] = reg_adder;
	transmit_data[1] = data;
	i2c_transmit(transmit_data, 2);
}
 
/**
 * @brief IIC 读取
 * @retval None
 */
void max30102_i2c_read(uint8_t reg_adder, uint8_t *pdata, uint8_t data_size)
{
	uint8_t adder = reg_adder;
	i2c_transmit(&adder, 1);
	i2c_receive(pdata, data_size);
}
 
/**
 * @brief max30102初始化
 * @retval None
 */
void max30102_init(void)
{
	uint8_t data;
 
	max30102_i2c_write(MODE_CONFIGURATION, 0x40); // reset the device
 
	delay_ms(5);
 
	max30102_i2c_write(INTERRUPT_ENABLE1, 0xE0);
	max30102_i2c_write(INTERRUPT_ENABLE2, 0x00); 	// interrupt enable: FIFO almost full flag, new FIFO Data Ready,
																								// ambient light cancellation overflow, power ready flag,
																								// internal temperature ready flag
 
	max30102_i2c_write(FIFO_WR_POINTER, 0x00);
	max30102_i2c_write(FIFO_OV_COUNTER, 0x00);
	max30102_i2c_write(FIFO_RD_POINTER, 0x00); 		// clear the pointer
 
	max30102_i2c_write(FIFO_CONFIGURATION, 0x4F); // FIFO configuration: sample averaging(1),FIFO rolls on full(0), FIFO almost full value(15 empty data samples when interrupt is issued)
 
	max30102_i2c_write(MODE_CONFIGURATION, 0x03); // MODE configuration:SpO2 mode
 
	max30102_i2c_write(SPO2_CONFIGURATION, 0x2A); // SpO2 configuration:ACD resolution:15.63pA,sample rate control:200Hz, LED pulse width:215 us
 
	max30102_i2c_write(LED1_PULSE_AMPLITUDE, 0x2f); // IR LED
	max30102_i2c_write(LED2_PULSE_AMPLITUDE, 0x2f); // RED LED current
 
	max30102_i2c_write(TEMPERATURE_CONFIG, 0x01); 	// temp
 
	max30102_i2c_read(INTERRUPT_STATUS1, &data, 1);
	max30102_i2c_read(INTERRUPT_STATUS2, &data, 1); // clear status
}
 
/**
 * @brief fifo区读取
 * @param output_data
 * @retval None
 */
void max30102_fifo_read(float *output_data)
{
	uint8_t receive_data[6];
	uint32_t data[2];
	max30102_i2c_read(FIFO_DATA, receive_data, 6);
	data[0] = ((receive_data[0] << 16 | receive_data[1] << 8 | receive_data[2]) & 0x03ffff);
	data[1] = ((receive_data[3] << 16 | receive_data[4] << 8 | receive_data[5]) & 0x03ffff);
	*output_data = data[0];
	*(output_data + 1) = data[1];
}
 
/**
 * @brief 获取心率
 * @param input_data cache_nums(缓存区的最大数字)
 * @retval (uint16_t)心率
 */
uint16_t max30102_getHeartRate(float *input_data, uint16_t cache_nums)
{
	float input_data_sum_aver = 0;
	uint16_t i, temp;
 
	for (i = 0; i < cache_nums; i++)
	{
		input_data_sum_aver += *(input_data + i);
	}
	input_data_sum_aver = input_data_sum_aver / cache_nums;
	for (i = 0; i < cache_nums; i++)
	{
		if ((*(input_data + i) > input_data_sum_aver) && (*(input_data + i + 1) < input_data_sum_aver))
		{
			temp = i;
			break;
		}
	}
	i++;
	for (; i < cache_nums; i++)
	{
		if ((*(input_data + i) > input_data_sum_aver) && (*(input_data + i + 1) < input_data_sum_aver))
		{
			temp = i - temp;
			break;
		}
	}
	if ((temp > 14) && (temp < 100))
	{
		return 3000 / temp;
	}
	else
	{
		return 0;
	}
}
 
/**
 * @brief 获取血氧
 * @param input_data red_input_data cache_nums(缓存区的最大数字)
 * @retval (float)血氧
 */
float max30102_getSpO2(float *ir_input_data, float *red_input_data, uint16_t cache_nums)
{
	float ir_max = *ir_input_data, ir_min = *ir_input_data;
	float red_max = *red_input_data, red_min = *red_input_data;
	float R;
	uint16_t i;
	for (i = 1; i < cache_nums; i++)
	{
		if (ir_max < *(ir_input_data + i))
		{
			ir_max = *(ir_input_data + i);
		}
		if (ir_min > *(ir_input_data + i))
		{
			ir_min = *(ir_input_data + i);
		}
		if (red_max < *(red_input_data + i))
		{
			red_max = *(red_input_data + i);
		}
		if (red_min > *(red_input_data + i))
		{
			red_min = *(red_input_data + i);
		}
	}
 
	R = ((ir_max + ir_min) * (red_max - red_min)) / ((red_max + red_min) * (ir_max - ir_min));
	return ((-45.060) * R * R + 30.354 * R + 94.845);
}
 
/**
 * @brief MAX30102服务函数
 * @param HeartRate(心率) SpO2(血氧) max30102_data fir_output
 * @retval (uint8_t)MAX30102_DATA_OK:结束读取  (uint8_t)!MAX30102_DATA_OK:还在读取
 */

uint8_t MAX30102_Get_DATA(uint16_t *HeartRate,float *SpO2,float max30102_data[2],float fir_output[2])
{
	if (max30102_int_flag) // 中断信号产生
	{
		max30102_int_flag = 0;
		max30102_fifo_read(max30102_data); // 读取数据
		ir_max30102_fir(&max30102_data[0], &fir_output[0]);
		red_max30102_fir(&max30102_data[1], &fir_output[1]);                                    // 滤波
		if ((max30102_data[0] > PPG_DATA_THRESHOLD) && (max30102_data[1] > PPG_DATA_THRESHOLD)) // 大于阈值，说明传感器有接触
		{
			ppg_data_cache_IR[cache_counter] = fir_output[0];
			ppg_data_cache_RED[cache_counter] = fir_output[1];
			cache_counter++;
		}
		else // 小于阈值
		{
			cache_counter = 0;
		}
		if (cache_counter >= CACHE_NUMS) // 收集满了数据
		{
			*HeartRate = max30102_getHeartRate(ppg_data_cache_IR, CACHE_NUMS);
			*SpO2 = max30102_getSpO2(ppg_data_cache_IR, ppg_data_cache_RED, CACHE_NUMS);
			cache_counter = 0;
			return MAX30102_DATA_OK;
		}
	}
	return !MAX30102_DATA_OK;
}
 
/**
 * @brief MAX30102输入引脚外部中断触发
 * @param GPIO_Pin
 * @attention cubemx配置下降沿 上拉 允许中断
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MAX30102_INT_Pin)
  {
    max30102_int_flag = 1;
  }
}
 

