#include "arm_const_structs.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>
#include "stdlib.h"

typedef struct Statistic_value
{
	float32_t Statistic_max;
	float32_t Statistic_min;
	float32_t Statistic_var;
	float32_t Statistic_rms;
	float32_t Statistic_mean;
	float32_t Statistic_std;
	float32_t Statistic_crestFactor;
	float32_t Statistic_kurtosis;
	float32_t Statistic_skewness;
	float32_t Statistic_FreqOvall;
	float32_t Statistic_SpeedOvall;
	float32_t Statistic_FreqPeak[20];
	float32_t Statistic_p2p;



	float32_t Statistic_max_Temp;
	float32_t Statistic_min_Temp;
	float32_t Statistic_var_Temp;
	float32_t Statistic_rms_Temp;
	float32_t Statistic_mean_Temp;
	float32_t Statistic_std_Temp;
	float32_t Statistic_crestFactor_Temp;
	float32_t Statistic_kurtosis_Temp;
	float32_t Statistic_skewness_Temp;
	float32_t Statistic_FreqOvall_Temp;

}Sv;
Sv statistic_value;
Sv Xstatistic_value;
Sv Ystatistic_value;
Sv Zstatistic_value;

float Calculate_max(float *data);
float Calculate_FreqMax(float *data,int freqSettingValue, int8_t freq_index);
float Calculate_skewness(float *data, int n);
float Calculate_kurtosis(float *data, int n);
float Calculate_rms(float *data, int n);
float Calculate_FreqOverAll(float *data, int n);
void Calculate_All_statisitc(float32_t *statisticDataSet, uint16_t dataLength, Sv * staticInstance);
void Calculate_FFT_RMS(float32_t * bufferforFFT, float32_t * OutputFFTbuffer, int32_t fftSize, Sv *staticInstance);



