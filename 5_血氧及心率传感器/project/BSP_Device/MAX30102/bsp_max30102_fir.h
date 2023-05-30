#ifndef __BSP_MAX30102_FIR_H_
#define __BSP_MAX30102_FIR_H_
 
#include "./math/arm_const_structs.h"

void max30102_fir_init(void);
void ir_max30102_fir(float *input,float *output);
void red_max30102_fir(float *input,float *output);

#endif /* __BSP_MAX30102_FIR_H_ */
 

