#include "Structs_header.h"
#include <Eigen/Dense>
#include <map>
#include <array>
#include <string>
#include <vector>
#include <iostream>
class KalmanFilter
{
public:
	KalmanFilter(std::map<std::string, std::array<double,3>> Values);

	void SetAngularVelocity(double w_x, double w_y, double w_z, long Time);

	void SetMagnetometerMeasurements(double m_x, double m_y, double m_z);

	void SetAccelerometerMeasurements(double a_x, double a_y, double a_z);



private:
	Eigen::Matrix<double, 7, 1> Xk;
	Eigen::Matrix<double, 7, 7> Pk;
	Eigen::Matrix<double, 7, 1> Xk_1;
	Eigen::Matrix<double, 7, 7> Pk_1;
	Eigen::Matrix<double, 7, 7> Q_k;
	Eigen::Matrix<double, 7, 7> R_k;
	Eigen::Matrix<double, 7, 1> RungeKuttaEval(Eigen::Matrix<double, 7, 1> X_0, double T, Eigen::Matrix3d w); // Kalman Filter 
	long previousT = 0;
	bool first_measurement_received = false;

	void set_mag_0(std::array<double, 3> values);
	void set_acc_0(std::array<double, 3> values);
	void set_mag_sig(std::array<double, 3> values);
	void set_acc_sig(std::array<double, 3> values);
	void set_gyro_drift_0(std::array<double, 3> values);
	void set_gyro_drift_sig(std::array<double, 3> values);
	void compute_initial_params();

	Eigen::Matrix3d ComputeTriad(Eigen::Vector3d m, Eigen::Vector3d s);

	Eigen::Vector4d ConvertRotatationMatrix2Quarternion(Eigen::Matrix3d R);

	Eigen::Vector3d Mag_0;
	Eigen::Vector3d Acc_0;
	std::array<double,6> R_Sigma;
	Eigen::Vector3d Gyro_drift_0;
	Eigen::Vector3d Gyro_Sigma;
	Eigen::Vector3d InitialTriad;
	
};

