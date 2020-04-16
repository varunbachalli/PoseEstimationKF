#pragma once
#include<Eigen/Dense>
#include <iostream>
#include <stdlib.h> 
#include <time.h> 
#include <math.h>
#include "Matrix_multiplication.cuh"
#include <limits>

#include <fstream>
#include <string>
#include <sstream>


struct Eigen_Vec_Vals
{
	Eigen::Matrix3d EigenValues;
	Eigen::Matrix3d EigenVectors;
};

class Magnetometer_Calibration
{
public:
	void setValues(double x, double y, double z);
	Magnetometer_Calibration(char SensorType);

	bool SamplesToBeCollected();
	Eigen::Vector3d CorrectValues(double x_, double y_, double z_);

	

	// temp public

	
private:
	void ClearHeapValues();
	Eigen_Vec_Vals JacobiMethod(Eigen::Matrix3d R);
	//Eigen::Matrix3d get_W();
	//Eigen::Vector3d get_bias();
	void setUncalibratedValues();
	bool IsDoubleZero(double a);
	const static int total_values = 1000;
	int num_rows = 0;
	Eigen::Vector3d bias;
	Eigen::Matrix3d W;
	double* UncalibratedValues = NULL; // *(UncalibratedValues + i * col + j)
	double* x = NULL;
	double* y = NULL;
	double* z = NULL;

	bool values_set = false;
	Eigen::Matrix3d LeastSquares_calculation(double* A);
	void setW();
	void LargestOffDiagonal(Eigen::Matrix3d A, int a[2]);
	

	char sensorType;



};

