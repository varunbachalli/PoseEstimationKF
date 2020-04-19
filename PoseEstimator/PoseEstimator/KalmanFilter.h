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

class KalmanFilter
{
public:
	KalmanFilter();
	KalmanFilter(std::map<std::string, std::array<double,3>> Values, long timestamp);
	void SetAngularVelocity(double w_x, double w_y, double w_z, long Time);
	void SetMagnetometerMeasurements(double m_x, double m_y, double m_z);
	void SetAccelerometerMeasurements(double a_x, double a_y, double a_z);
	Eigen::Vector4d getXk_1();
	Eigen::Vector3d getRPY_1(Eigen::Vector4d Quart);
	Eigen::Vector3d getRPY();
private:
	Eigen::Matrix<double, 4, 1> X_k;
	Eigen::Matrix<double, 4, 4> P_k;
	Eigen::Matrix<double, 4, 4> S_k;
	Eigen::Matrix<double, 4, 4> Q_k;
	Eigen::Matrix<double, 4, 4> R_k;
	Eigen::Matrix<double, 4, 4> K_k;
	Eigen::Vector4d z_k;


	Eigen::Vector4d RungeKuttaEval(Eigen::Matrix<double, 4, 1> X_0, double T, Eigen::Vector3d U0); // Kalman Filter 
	long previousT = 0;
	bool first_measurement_received = false;
	Eigen::Matrix4d  GetJacobian(Eigen::Vector3d U0);

	void set_mag_0(std::array<double, 3> values);
	void set_acc_0(std::array<double, 3> values);
	void set_mag_sig(std::array<double, 3> values);
	void set_acc_sig(std::array<double, 3> values);
	void set_gyro_drift_0(std::array<double, 3> values);
	void set_gyro_drift_sig(std::array<double, 3> values);
	void compute_initial_params();

	Eigen::Matrix3d ComputeTriad(Eigen::Vector3d m, Eigen::Vector3d s);

	Eigen::Vector4d RotatationMatrix2Quarternion(Eigen::Matrix3d R);

	void Prediction(Eigen::Vector3d Gyro, long T);

	void Correction();

	
	std::array<double,6> R_Sigma;
	Eigen::Vector3d Gyro_drift_0;
	Eigen::Vector3d Gyro_Sigma;
	Eigen::Matrix3d InitialTriad;

	Eigen::Vector3d Mag_0;
	Eigen::Vector3d Acc_0;
	Eigen::Vector3d Mag_1;
	Eigen::Vector3d Acc_1;
	/*Eigen::Vector3d Gyro_w;*/

	int numberofPrints = 0;
	int min = 100;
	int max = 125;

	
};

