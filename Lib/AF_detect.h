#ifndef __AF_DETECT_H
#define __AF_DETECT_H

#include "main.h"

// Simple Func
uint8_t getMax(uint8_t *n);
uint8_t getMin(uint8_t *n);
float getMean(uint8_t *n);
float getMean_f(float *n);
int16_t getSum(int16_t *n);
uint8_t getSum_u(uint8_t *n);

// AF func
float AF_Normlaized_RMSSD(uint8_t RRI[]); // 0.13
float AF_Shannon_Entropy(uint8_t RRI[]); // 0.76
float AF_Sample_Entropy(uint8_t RRI[]); // 1.3
float AF_Kurtosis(uint8_t RRI[]); // 5.2
float AF_Turning_Point_Ratio(uint8_t RRI[]); // 0.33
uint8_t AF_Peak_detection(float filtered_ppgDataF[], uint8_t RRI[]); 

#endif