#ifndef __FIR_CONSTANTS_H
#define __FIR_CONSTANTS_H

//rri
#define NUM_TAPS 100
#define SAMPLE_RATE 50
#define FILTERING_RATE 20 // 2sec
#define BLOCK_SIZE 400
#define BUFF_SIZE BLOCK_SIZE * 2
#define RRI_COUNT 17 // RRI +1
#define SAMPLE_N 16

//fft
#define NUM_TAPS_FFT 50
#define SAMPLE_RATE_FFT 25
#define FILTERING_RATE_FFT 20 // 1sec
#define BLOCK_SIZE_FFT 250
#define BUFF_SIZE_FFT BLOCK_SIZE_FFT * 2
#define NFFT 2048
#define USING_FFT 222


#endif	