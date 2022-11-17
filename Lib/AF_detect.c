#include "AF_detect.h"

#define SAMPLE_N 16
#define NUM_TAPS 100
#define SAMPLE_RATE 50
#define FILTERING_RATE 50 
#define BLOCK_SIZE 400
#define BUFF_SIZE BLOCK_SIZE * 2

uint8_t getMax(uint8_t *n){
	uint8_t max = n[0];
	
	for (uint8_t i = 1; i < SAMPLE_N; i++)
		if(n[i] > max) max = n[i];
	
	return max;
}
uint8_t getMin(uint8_t *n){
	uint8_t min = n[0];
	
	for (uint8_t i = 1; i < SAMPLE_N; i++)
		if(n[i] < min) min = n[i];
	
	return min;
}
float getMean(uint8_t *n){
	int16_t sum = 0;
	float mean = 0;
	
	for (uint8_t i = 0; i < SAMPLE_N; i++)
		sum += n[i];
	mean = ((float)sum/(float)SAMPLE_N);
	
	return mean;
}

float getMean_f(float *n){
	float sum = 0;
	float mean = 0;
	
	for (uint8_t i = 0; i < SAMPLE_N; i++)
		sum += n[i];
	mean = sum/SAMPLE_N;
	
	return mean;
}
int16_t getSum(int16_t *n){
	int16_t sum=0;
	
	for (uint8_t i = 0; i < SAMPLE_N-1; i++)
		sum += n[i];
	
	return sum;
}

uint8_t getSum_u(uint8_t *n){
	uint8_t sum=0;
	
	for (uint8_t i = 0; i < SAMPLE_N-1; i++)
		sum += n[i];
	
	return sum;
}

float AF_Normlaized_RMSSD(uint8_t RRI[]){
	
	int16_t tmp[SAMPLE_N - 1] = {0,};
	float rmssd;
	
	for(uint8_t i = 1;i<SAMPLE_N;i++){
		tmp[i-1] = RRI[i] - RRI[i-1];
		tmp[i-1] = pow(tmp[i-1],2);
	}
	
	rmssd = sqrt(((float)getSum(tmp)/(float)(SAMPLE_N-1)));
	rmssd /= getMean(RRI);

	return rmssd;
}
float AF_Shannon_Entropy(uint8_t RRI[]){
	uint8_t RRI_max = getMax(RRI);
	uint8_t RRI_min= getMin(RRI);
	uint8_t RRI_group[SAMPLE_N] = {0,};
	uint8_t tmp;
	float shaanon_E = 0;
	float norm_group[SAMPLE_N]={0,};
	
	float RRI_step = (float) (RRI_max-RRI_min)/SAMPLE_N;
	
	if(RRI_step == 0)
		return RRI_step;
	else{
		for(uint8_t i = 0;i<SAMPLE_N;i++){
			tmp = round((float)(RRI[i]-RRI_min)/RRI_step);
			RRI_group[tmp]++;
		}
		for(uint8_t i = 0;i<SAMPLE_N;i++)
			norm_group[i] = (float)RRI_group[i]/(float)SAMPLE_N;
		
		for(uint8_t i = 0;i<SAMPLE_N;i++)
			if(norm_group[i]>0)
				shaanon_E+=(norm_group[i]*logf(norm_group[i]));
	}
	
	shaanon_E/=logf((float)1/(float)SAMPLE_N);
	return shaanon_E;
}
float AF_Sample_Entropy(uint8_t RRI[]){
	uint8_t m = 1;
	float r = 0.06, xm[SAMPLE_N-1]={0,}, xm1[2][SAMPLE_N-1]={0,};
	float RRI_mean = getMean(RRI), sample_entropy;
	uint16_t Sum_Nm1 = 0, Sum_Nm2 = 0; 
	
	for(uint8_t i = 0;i<SAMPLE_N-m;i++){
		xm[i] = RRI[i+m-1]/RRI_mean;
		for(uint8_t j = 0;j<m+1;j++){ 
			xm1[j][i] = RRI[i+j]/RRI_mean;
		}		
	}
	
	for(uint8_t i = 0;i<SAMPLE_N-m;i++){
		uint8_t comp1[SAMPLE_N-1]={0,},comp2[SAMPLE_N-1]={0,};
		for(uint8_t j = 0;j<SAMPLE_N-m;j++){
			if(j!=i){
				float d1=0,d2=0;
				d1 = fabs(xm[i]-xm[j]);
				for(uint8_t z = 0;z<m+1;z++)
					d2 += pow((xm1[z][i]-xm1[z][j]),2);
				d2 = sqrt(d2);	
				
				if (d1<=r)
					comp1[j]=1;
				if (d2<=r)
					comp2[j]=1;
			}
		}
		Sum_Nm1 += getSum_u(comp1);
		Sum_Nm2 += getSum_u(comp2);
	}
		
	if(Sum_Nm1==0)
		sample_entropy=0;
	else if(Sum_Nm2 == 0)
		sample_entropy=-logf((float)1/(float)Sum_Nm1);
	else
		sample_entropy=-logf((float)Sum_Nm2/(float)Sum_Nm1);
	
	return sample_entropy;
}
float AF_Kurtosis(uint8_t RRI[]){
	float mean_data = getMean(RRI);
	float a[SAMPLE_N], b[SAMPLE_N], c[SAMPLE_N], kurtosis;
		
	for(uint8_t i=0;i<SAMPLE_N;i++){
		a[i] = RRI[i] - mean_data;
		b[i] = pow(a[i],2);
		c[i] = pow(a[i],4);
	}
	b[0] = getMean_f(b);
	c[0] = getMean_f(c);
	
	kurtosis = c[0]/pow(b[0],2);
	return kurtosis;
}

float AF_Turning_Point_Ratio(uint8_t RRI[]){
	uint8_t cnt = 0;
	float tpr;
	
	for(uint8_t i=1;i<SAMPLE_N-1;i++){
		if(RRI[i] > RRI[i-1] && RRI[i] > RRI[i+1])
			cnt++;
		else if(RRI[i] < RRI[i-1] && RRI[i] < RRI[i+1])
			cnt++;
	}
	
	tpr = (float)cnt/(float)60;
	
	return tpr;
}
uint8_t AF_Peak_detection(float filtered_ppgDataF[], uint8_t RRI[]){
	uint8_t cnt=0;
	uint16_t peak_idx[100]={0,};
	float peak_amp[100]={0,};
	float mean = 0;
	uint8_t sec_thres = SAMPLE_RATE*0.3;
	uint8_t mountain_count = 0;
	
	//moving average filter
//	uint8_t avr_num = SAMPLE_RATE/10;
//	uint16_t avr_length = BLOCK_SIZE-NUM_TAPS-avr_num;
//	float avr_ppg[BLOCK_SIZE-NUM_TAPS] = {0,};
	
//	for(int i=0;i<=avr_length;i++)
//		avr_ppg[i] = (float)(filtered_ppgDataF[i+NUM_TAPS]+filtered_ppgDataF[i+1+NUM_TAPS]+filtered_ppgDataF[i+2+NUM_TAPS]+filtered_ppgDataF[i+3+NUM_TAPS]+filtered_ppgDataF[i+4+NUM_TAPS])/((float)avr_num);
		
	//find all peaks
//	for(int i=1;i<avr_length;i++){
//		if(avr_ppg[i-1] < avr_ppg[i] && avr_ppg[i] > avr_ppg[i+1]){
//			peak_idx[cnt] = i;
//			peak_amp[cnt] = avr_ppg[i];
//			cnt++;
//		}
//		mean += avr_ppg[i-1]/(float)(avr_length+1);
//	}
//	mean += avr_ppg[avr_length]/(float)(avr_length+1);
	
	for(int i=NUM_TAPS;i<BLOCK_SIZE-1;i++){
		if(filtered_ppgDataF[i-1] < filtered_ppgDataF[i]){
			mountain_count++;
			if(filtered_ppgDataF[i] > filtered_ppgDataF[i+1] && mountain_count > SAMPLE_RATE*0.1 && filtered_ppgDataF[i] > 0){
				peak_idx[cnt] = i;
				peak_amp[cnt] = filtered_ppgDataF[i];
				cnt++;
			}
		}
		else
			mountain_count=0;
	}
	
	// 0.3 second
	for(int i=0;i<cnt-1;i++){
		if(peak_idx[i+1] - peak_idx[i] <= sec_thres){
			int j;
			if(peak_amp[i] > peak_amp[i+1])
				j = i+1;
			else
				j = i;
			
			for(;j<cnt-1;j++)
			{
				peak_amp[j] = peak_amp[j+1];
				peak_idx[j] = peak_idx[j+1];
			}
			peak_amp[cnt-1] = 0;
			peak_idx[cnt-1] = 0;
			cnt--;
		}
	}
	
	//return RRI
	uint8_t RRI_cnt = 0;
	for(int i=0;i<cnt-1;i++){
		RRI[RRI_cnt] = peak_idx[i+1] - peak_idx[i];
		RRI_cnt++;		
	}
	return RRI_cnt;
}

