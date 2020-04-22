#include "InitialValues.h"


InitialValues::InitialValues(){}
InitialValues::InitialValues(int numvalues)
{
	x  = (double*)	malloc(sizeof(double)* numvalues);
	y  = (double*)	malloc(sizeof(double)* numvalues);
	z  = (double*)	malloc(sizeof(double)* numvalues);
	n_avg_values = numvalues;
}

void InitialValues::setValuesforAverage(double x_, double y_, double z_)
{
	x[n] = x_;
	y[n] = y_;
	z[n] = z_;

	avg[0] += x_;
	avg[1] += y_;
	avg[2] += z_;
	n++;

	if (n_avg_values == n)
	{
		std::cout << n << " values recieved for the sensor " << sensorType << " and calibration is done" <<std::endl;
		compute_mean_and_variance();
		isCalibrated = true;
	}

}

void InitialValues::getAverageValues(double average[3]) // when calling clear the pointer.
{
	average = avg;
}

void InitialValues::getVariance(double variance[3])
{	
	variance = var;
}

void InitialValues::compute_mean_and_variance()
{
	avg[0] = avg[0] / n_avg_values;
	avg[1] = avg[1] / n_avg_values;
	avg[2] = avg[2] / n_avg_values;

	for (int i = 0; i < n_avg_values; ++i)
	{
		var[0] += (x[i] - avg[0]) * (x[i] - avg[0]);
		var[1] += (y[i] - avg[1]) * (y[i] - avg[1]);
		var[2] += (z[i] - avg[2]) * (z[i] - avg[2]);
	}
	
	var[0] = var[0]/n_avg_values;
	var[1] = var[1]/n_avg_values;
	var[2] = var[2]/n_avg_values;
}

bool InitialValues::sensorCalibrated()
{
	return isCalibrated;
}

