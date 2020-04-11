#pragma once
#include<Eigen/Dense>
#include <iostream>
#include <stdlib.h> 
#include<time.h> 
#include<math.h>

class Calibration
{
public:
	void setUncalibratedValues(double x, double y, double z);
	Eigen::Matrix3d return_W();
	Eigen::Vector3d return_bias();
	void setUncalibratedValues();
	Calibration();
	~Calibration();
	const static int total_values = 2;
	void returnA_Atranspose(double A[][6], double R[6][6]);
	void returnAtranspose_b(double A[][6], double b[6], int size);
	void setW();
private:
	
	int num_rows = 0;
	Eigen::Vector3d bias;
	Eigen::Matrix3d W;
	//Eigen::Matrix<double, total_values, 3> UncalibratedValues;
	int cols = 6;
	int rows = 1000;
	double* UncalibratedValues; // *(UncalibratedValues + i * col + j)
	double* x;
	double* y;
	double* z;
	bool values_set = false;
	void Set_R_LeastSquares(Eigen::Matrix<double, Calibration::total_values, 6>& A);
};

