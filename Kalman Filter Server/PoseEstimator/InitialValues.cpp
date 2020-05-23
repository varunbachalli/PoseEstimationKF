#include "InitialValues.h"


InitialValues::InitialValues(){}
InitialValues::InitialValues(int numvalues)
{
	x = (double*)malloc(sizeof(double) * numvalues);
	y = (double*)malloc(sizeof(double) * numvalues);
	z = (double*)malloc(sizeof(double) * numvalues);
	avg[0] = 0.0;
	avg[1] = 0.0;
	avg[2] = 0.0;
	var[0] = 0.0;
	var[1] = 0.0;
	var[2] = 0.0;
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
		compute_mean_and_variance();
		isCalibrated = true;
	}

}

std::array<double, 3> InitialValues::getAverageValues() // when calling clear the pointer.
{
	return avg;
}

std::array<double, 3> InitialValues::getVariance()
{
	return var;
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
	
	

	var[0] = var[0]/(double)(n_avg_values-1);
	var[1] = var[1]/(double)(n_avg_values-1);
	var[2] = var[2]/(double)(n_avg_values-1);

	printf("variance [%f,%f,%f]\n", var[0], var[1], var[2]);
	printf("average [%f,%f,%f]\n", avg[0], avg[1], avg[2]);

	free(x);
	free(y);
	free(z);
}

bool InitialValues::sensorCalibrated()
{
	return isCalibrated;
}

