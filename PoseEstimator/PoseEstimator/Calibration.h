#pragma once
#include<Eigen/Dense>
#include <iostream>
#include <stdlib.h> 
#include <time.h> 
#include <math.h>
#include "Matrix_multiplication.cuh"
#include <limits>

struct Eigen_Vec_Vals
{
	Eigen::Matrix3d EigenValues;
	Eigen::Matrix3d EigenVectors;
};

class Calibration
{
public:
	void setValues(double x, double y, double z);
	Eigen::Matrix3d get_W();
	Eigen::Vector3d get_bias();
	Calibration();
	~Calibration();
	bool SamplesToBeCollected();


	// temp public

	Eigen_Vec_Vals JacobiMethod(Eigen::Matrix3d R);
private:
	void setUncalibratedValues();
	bool IsZero(double a);
	
	const static int total_values = 1000;
	int num_rows = 0;
	Eigen::Vector3d bias;
	Eigen::Matrix3d W;
	double* UncalibratedValues; // *(UncalibratedValues + i * col + j)
	double* x;
	double* y;
	double* z;
	bool values_set = false;
	Eigen::Matrix3d LeastSquares_calculation(double* A);
	void setW();
	void LargestOffDiagonal(Eigen::Matrix3d A, int a[2]);
	const static int pi = 3.14159265358979323846;

};

