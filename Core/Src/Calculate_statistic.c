#include "Calculate_statistic.h"



float Calculate_rms(float *data, int n)
{
	float32_t rmsAns = 0;
	float32_t * ans = &rmsAns;
	float32_t sum = 0;

	for (int i = 0; i < n; i++)
		sum += data[i]*data[i];

	sum = sum / n;
	arm_sqrt_f32(sum, ans);


	return *ans;
}

float Calculate_max(float *data)
{
	float max= 0;
	float mid= 0;  //
		for(int i = 0;i<4096;i++){
		/*因為arr[i]是固定的 ，所以可以用arr[i]先和max比較，如果大於max 則
			把max的值賦給mid的值，然後把arr[i]賦給max  這樣max肯定比mid大*/
		if(data[i]>max){
		mid = max;
		max =data[i];

			}else if(data[i]>mid){/*  另一種情況 arr[i]處於兩者之間*/
				mid = data[i];//  則吧arr[i]賦給mid
			}


		}
		printf("max=%d,mid=%d\n",max,mid);


		return max;
}
float Calculate_FreqMax(float *data, int freqSettingValue, int8_t freq_index)
{
	if(freqSettingValue != 0)
	{
		volatile float temp = 0;
			if(freqSettingValue < 25)
			{
				for(int i = freqSettingValue- 25; i<freqSettingValue + 25; i++)
					{

						if(data[i] > temp)
						{
							temp += data[i];
						}
					}
			}
			else
			{

				for(int i = freqSettingValue; i<freqSettingValue + 25; i++)
					{

						if(data[i] > temp)
						{
							temp += data[i];
						}
					}
			}
		statistic_value.Statistic_FreqPeak[freq_index] = temp;
	}

}



/*
 *
 * Calculate Skewness & kurtosis
 *
 * */
float CalcluateMean(float *x, int n)
{
	float sum = 0;

	for (int i = 0; i < n; i++)
		sum += x[i];

	return sum / n;
}

float CalcluateMoment(float *x, int n, int m)
{
	float mean = CalcluateMean(x, n), sum = 0;

	for (int i = 0; i < n; i++)
		sum += pow(x[i] - mean, m);

	return sum / n;
}

float CalcluateVariance(float *x, int n)
{
	float mean = CalcluateMean(x, n), sumSq = 0;

	for (int i = 0; i < n; i++)
	{
		float delta = x[i] - mean;

		sumSq += delta * delta;
	}

	return sumSq / (n - 1);
}
float CalcluateStandardDeviation(float *x, int n)
{
	return sqrt(CalcluateVariance(x, n));
}

float Calculate_skewness(float *x, int n)
{
	// NIST definition of adjusted Fisher-Pearson
	// coefficient of skewness
	float m3 = CalcluateMoment(x, n, 3);
	float sx = CalcluateStandardDeviation(x, n);
	int n1 = n - 1;
	float Skewness = (sqrt(n * n1) / n1) * m3 / pow(sx, 3);

	return Skewness;
}
float Calculate_kurtosis(float *x, int n)
{
	float m2 = CalcluateMoment(x, n, 2);
	float m4 = CalcluateMoment(x, n, 4);


	float Kurtosis = m4 / (m2 * m2) - 3.0;
    return Kurtosis;
}

float Calculate_FreqOverAll(float *x, int n)
{
	float ParsevalFftPower = 0;
	float ans = 0;
	for(int i = 0; i<n; i++)
	{
		//FFTRMSArray[i] = (testOutput[i]*2)/4096;
		ParsevalFftPower += x[i] * x[i];
	}

	ans = sqrt(ParsevalFftPower)/n;


	return ans;
}

void Calculate_All_statisitc(float32_t *statisticDataSet, uint16_t dataLength, Sv *staticInstance)
{
	uint32_t maxtestIndex = 0;
	uint32_t mintestIndex = 0;

	arm_max_f32(statisticDataSet, dataLength, &staticInstance->Statistic_max, &maxtestIndex);
	arm_min_f32(statisticDataSet, dataLength, &staticInstance->Statistic_min, &mintestIndex);
	arm_rms_f32(statisticDataSet, dataLength, &staticInstance->Statistic_rms);
	staticInstance->Statistic_p2p = 0.92 * (staticInstance->Statistic_max - staticInstance->Statistic_min);

}

void Calculate_FFT_RMS(float32_t * bufferforFFT, float32_t * OutputFFTbuffer, int32_t fftSize, Sv *staticInstance)
{
	float32_t maxValue = 0;
	float32_t Speeddatabuffer[2048];
	uint32_t testIndex = 0;
	uint32_t ifftFlag = 0;
	uint32_t doBitReverse = 1;

	arm_cfft_f32(&arm_cfft_sR_f32_len2048, bufferforFFT, ifftFlag, doBitReverse);

	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(bufferforFFT, OutputFFTbuffer, fftSize);

	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(OutputFFTbuffer, fftSize, &maxValue, &testIndex);


	for(uint16_t i = 0; i < fftSize; i++)
	{
		Speeddatabuffer[i] = OutputFFTbuffer[i];
	}

	//TODO: remove low frequency noise (DC component)
	OutputFFTbuffer[0] = 0;
	OutputFFTbuffer[1] = 0;
	OutputFFTbuffer[2] = 0;

	staticInstance->Statistic_FreqOvall = Calculate_FreqOverAll(OutputFFTbuffer,2048);


	/*
	 * Compute Speed Ovall
	 *
	 * */
	float32_t sampleCount = 2048;
	float32_t samplingRate = 3200;
	float32_t frequencyScale = samplingRate/sampleCount;

	for(uint16_t i = 1; i < fftSize; i++)
	{
		if(i < fftSize/2)
		{
			if(i ==0)
			{
				Speeddatabuffer[i] = Speeddatabuffer[i];
			}
			else
			{
				Speeddatabuffer[i] = (Speeddatabuffer[i] * 9807) / (2 * 3.1415926 * frequencyScale * i);
			}

		}
		else if(i > fftSize/2)
		{
			Speeddatabuffer[i] = (Speeddatabuffer[i] * 9807) / (2 * 3.1415926 * frequencyScale * abs(fftSize-i));
		}

	}

	Speeddatabuffer[0] = 0;
	Speeddatabuffer[1] = 0;
	Speeddatabuffer[2] = 0;
	Speeddatabuffer[3] = 0;
	Speeddatabuffer[4] = 0;




	Speeddatabuffer[2043] = 0;
	Speeddatabuffer[2044] = 0;
	Speeddatabuffer[2045] = 0;
	Speeddatabuffer[2046] = 0;
	Speeddatabuffer[2047] = 0;


	staticInstance->Statistic_SpeedOvall = Calculate_FreqOverAll(Speeddatabuffer,2048);

}


