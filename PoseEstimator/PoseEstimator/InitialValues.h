#pragma once

#include<stdio.h>
#include <iostream>

#include <map>
#include <vector>

class InitialValues
{
public:
	InitialValues();
	InitialValues(int numvalues);
	void setValuesforAverage(double x_, double y_, double z_);
	void getAverageValues(double average[3]);
	void getVariance(double variance[3]);
	bool sensorCalibrated();
private:

	void compute_mean_and_variance();
	bool isCalibrated = false;
	char sensorType;
	int n = 0;
	int n_avg_values = 300;
	double avg[3];
	double var[3];
	double* x = NULL;
	double* y = NULL;
	double* z = NULL;
};

