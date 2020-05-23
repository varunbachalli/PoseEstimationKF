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
	Magnetometer_Calibration();

	bool MagnetometerCalibrated();
	void CorrectValues(double& x_, double& y_, double& z_);
	static Eigen_Vec_Vals JacobiMethod(Eigen::Matrix3d R);
	static bool IsDoubleZero(double a);
	static void LargestOffDiagonal(Eigen::Matrix3d A, int a[2]);
	
private:
	void ClearHeapValues();

	void setUncalibratedValues();
	
	int numberOfEvaluationsJacobi = 0;
	const static int total_values = 100;
	int num_rows = 0;
	Eigen::Vector3d bias = Eigen::Vector3d::Zero();
	Eigen::Matrix3d W;
	double* UncalibratedValues = NULL; // *(UncalibratedValues + i * col + j)
	double* x = NULL;
	double* y = NULL;
	double* z = NULL;

	bool isCalibrated = false;
	Eigen::Matrix3d LeastSquares_calculation(double* A);
	void setW();
	
};

