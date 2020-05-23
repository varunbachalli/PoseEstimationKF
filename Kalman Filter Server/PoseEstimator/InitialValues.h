#pragma once

#include<stdio.h>
#include <iostream>

#include <map>
#include <vector>
#include <array>

class InitialValues
{
public:
	InitialValues();
	InitialValues(int numvalues);
	void setValuesforAverage(double x_, double y_, double z_);
	std::array<double, 3> getAverageValues();
	std::array<double, 3> getVariance();
	bool sensorCalibrated();
private:

	void compute_mean_and_variance();
	bool isCalibrated = false;
	char sensorType;
	int n = 0;
	int n_avg_values = 300;
	std::array<double,3> avg;
	std::array<double, 3> var;
	double* x = NULL;
	double* y = NULL;
	double* z = NULL;
};

