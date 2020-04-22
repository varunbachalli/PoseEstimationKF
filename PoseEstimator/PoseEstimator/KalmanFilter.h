#include "Structs_header.h"
#include <Eigen/Dense>
#include <map>
#include <array>
#include <string>
#include <vector>
#include <iostream>
#include <math.h> 
#include <conio.h>


// CSV imports

#include <fstream>
#include <string>
#include <sstream>


// Noise Addition

#include <stdlib.h> 
#include<time.h> 

class KalmanFilter
{
public:
	KalmanFilter();
	KalmanFilter(std::map<std::string, std::array<double,3>> Values, long long timestamp);
	void SetAngularVelocity(double w_x, double w_y, double w_z, long long timestamp);
	void SetMagnetometerMeasurements(double m_x, double m_y, double m_z);
	void SetAccelerometerMeasurements(double a_x, double a_y, double a_z);
	void getXk_1(double* quart);
	void getRPY_1(Eigen::Vector4d q, Eigen::Vector3d& angles);
	void getRPY(double* angles);
	void UpdateLatestPreviousTime(long long Time);
private:
	Eigen::Matrix<double, 4, 1> X_k;
	Eigen::Matrix<double, 4, 4> P_k;
	Eigen::Matrix<double, 4, 4> S_k;
	Eigen::Matrix<double, 4, 4> Q_k;
	Eigen::Matrix<double, 4, 4> R_k;
	Eigen::Matrix<double, 4, 4> K_k;
	Eigen::Vector4d z_k;


	void RungeKuttaEval(Eigen::Vector4d& q0, double T, Eigen::Vector3d U0); // Kalman Filter 
	long long previousT = 0;
	bool first_measurement_received = false;
	void GetJacobian(Eigen::Vector3d U0, Eigen::Matrix4d& Jacobian);

	void set_mag_0(std::array<double, 3> values);
	void set_acc_0(std::array<double, 3> values);
	void set_mag_sig(std::array<double, 3> values);
	void set_acc_sig(std::array<double, 3> values);
	void set_gyro_drift_0(std::array<double, 3> values);
	void set_gyro_drift_sig(std::array<double, 3> values);
	void compute_initial_params();

	void ComputeTriad(Eigen::Vector3d s, Eigen::Vector3d m, Eigen::Matrix3d& Triad);

	void RotatationMatrix2Quarternion(Eigen::Matrix3d a, Eigen::Vector4d& Quarternion);

	void Prediction(Eigen::Vector3d Gyro, long long T);

	void Correction();

	
	std::array<double,6> R_Sigma;
	Eigen::Vector3d Gyro_drift_0;
	Eigen::Vector3d Gyro_Sigma;
	Eigen::Matrix3d InitialTriad;

	Eigen::Vector3d Mag_0;
	Eigen::Vector3d Acc_0;
	Eigen::Vector3d Mag_1;
	Eigen::Vector3d Acc_1;
	Eigen::Vector3d Gyro_w;

	int numberofPrints = 0;
	int min = 100;
	int max = 125;

	
	
};

