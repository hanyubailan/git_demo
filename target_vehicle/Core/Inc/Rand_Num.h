#ifndef Rand_Num_H
#define Rand_Num_H

#include "stdio.h"
#include "stdlib.h"  
#include "time.h"   
#include "stm32f4xx_hal.h"
#include "math.h"
#define MAX_absRPM 3000

extern int32_t generate_random_change(void);
extern int32_t abs_ChangeNum_value(int32_t current_value);
extern int16_t RandRPM;

#endif
