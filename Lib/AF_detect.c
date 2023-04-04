#include "AF_detect.h"

float32_t AF_Normlaized_RMSSD(float32_t RRI[], float32_t RRI_mean){
	
	float32_t tmp[SAMPLE_N - 1] = {0,},sum_tmp = 0, rmssd;
	
	for(uint8_t i = 1;i<SAMPLE_N;i++){
		tmp[i-1] = RRI[i] - RRI[i-1];
		tmp[i-1] = pow(tmp[i-1],2);
		sum_tmp += tmp[i-1];
	}
	
	rmssd = sqrt(((float32_t)(sum_tmp)/(float32_t)(SAMPLE_N-1)));
	rmssd /= RRI_mean;

	return rmssd;
}
float32_t AF_Shannon_Entropy(float32_t RRI[]){
	uint8_t RRI_group[SAMPLE_N] = {0,};
	uint8_t tmp;
	float32_t shaanon_E = 0, norm_group[SAMPLE_N]={0,}, RRI_max, RRI_min;
	arm_max_f32(RRI,SAMPLE_N,&RRI_max,0);
	arm_min_f32(RRI,SAMPLE_N,&RRI_min,0);
	
	float32_t RRI_step = (float32_t) (RRI_max-RRI_min)/SAMPLE_N;
	
	if(RRI_step == 0)
		return RRI_step;
	else{
		for(uint8_t i = 0;i<SAMPLE_N;i++){
			tmp = round((float32_t)(RRI[i]-RRI_min)/RRI_step);
			RRI_group[tmp]++;
		}
		for(uint8_t i = 0;i<SAMPLE_N;i++)
			norm_group[i] = (float32_t)RRI_group[i]/(float32_t)SAMPLE_N;
		
		for(uint8_t i = 0;i<SAMPLE_N;i++)
			if(norm_group[i]>0)
				shaanon_E+=(norm_group[i]*logf(norm_group[i]));
	}
	
	shaanon_E/=logf((float32_t)1/(float32_t)SAMPLE_N);
	return shaanon_E;
}
float32_t AF_Sample_Entropy(float32_t RRI[], float32_t RRI_mean){
	uint8_t m = 1;
	float32_t r = 0.06, xm[SAMPLE_N-1]={0,}, xm1[2][SAMPLE_N-1]={0,};
	float32_t sample_entropy;
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
		for(int j = 0; j < SAMPLE_N; j++){
			Sum_Nm1 += comp1[j];
			Sum_Nm2 += comp2[j];
		}
	}
		
	if(Sum_Nm1==0)
		sample_entropy=0;
	else if(Sum_Nm2 == 0)
		sample_entropy=-logf((float32_t)1/(float32_t)Sum_Nm1);
	else
		sample_entropy=-logf((float32_t)Sum_Nm2/(float32_t)Sum_Nm1);
	
	return sample_entropy;
}
float32_t AF_Kurtosis(float32_t RRI[], float32_t mean_data){
	float32_t a[SAMPLE_N], b[SAMPLE_N], c[SAMPLE_N], kurtosis;
		
	for(uint8_t i=0;i<SAMPLE_N;i++){
		a[i] = RRI[i] - mean_data;
		b[i] = pow(a[i],2);
		c[i] = pow(a[i],4);
	}
	float32_t mean_b, mean_c;
	arm_mean_f32(b,SAMPLE_N,&mean_b);
	arm_mean_f32(c,SAMPLE_N,&mean_c);
	
	kurtosis = mean_c/pow(mean_b,2);
	return kurtosis;
}

float32_t AF_Turning_Point_Ratio(float32_t RRI[]){
	uint8_t cnt = 0;
	float32_t tpr;
	
	for(uint8_t i=1;i<SAMPLE_N-1;i++){
		if(RRI[i] > RRI[i-1] && RRI[i] > RRI[i+1])
			cnt++;
		else if(RRI[i] < RRI[i-1] && RRI[i] < RRI[i+1])
			cnt++;
	}
	
	tpr = (float)cnt/(float)60;
	
	return tpr;
}
uint8_t AF_Peak_detection(float32_t filtered_ppgDataF[], int16_t RRI[]){
	uint8_t cnt=0;
	uint16_t peak_idx[100]={0,};
	float32_t peak_amp[100]={0,};
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

