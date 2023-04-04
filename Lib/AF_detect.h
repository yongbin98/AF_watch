#ifndef __AF_DETECT_H
#define __AF_DETECT_H

#include "main.h"
#include "arm_math.h"
#include "fir_constants.h"

// AF func
float32_t AF_Normlaized_RMSSD(float32_t RRI[], float32_t RRI_mean); // 0.13
float32_t AF_Shannon_Entropy(float32_t RRI[]); // 0.76
float32_t AF_Sample_Entropy(float32_t RRI[], float32_t RRI_mean); // 1.3
float32_t AF_Kurtosis(float32_t RRI[], float32_t mean_data); // 5.2
float32_t AF_Turning_Point_Ratio(float32_t RRI[]); // 0.33
uint8_t AF_Peak_detection(float filtered_ppgDataF[], int16_t RRI[]); 

#endif