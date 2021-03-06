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
	Eigen::Matrix<double, 3, 3> Q_k;
	Eigen::Matrix<double, 4, 4> R_k;
	Eigen::Matrix<double, 4, 4> K_k;
	Eigen::Vector4d z_k;

	
	void RungeKuttaEval(Eigen::Vector4d& q0, long long T, Eigen::Vector3d U0);
	void WriteTextFile(std::string text);
	long long previousT = 0;
	bool first_measurement_received = false;
	void GetJacobian_A(Eigen::Vector3d U0, Eigen::Matrix4d& Jacobian);
	void GetJacobian_B(Eigen::Vector4d q, Eigen::Matrix<double, 4, 3>& Jacobian);
	void LowPassFilter(Eigen::Vector3d Reading, Eigen::Vector3d& y, double alpha);
	void set_mag_0(std::array<double, 3> values);
	void set_acc_0(std::array<double, 3> values);

	Eigen::Matrix3d WahbaSVDRotation(Eigen::Vector3d Mag, Eigen::Vector3d Acc, double k_acc, double k_mag);

	void compute_initial_params();

	void ComputeTriad(Eigen::Vector3d s, Eigen::Vector3d m, Eigen::Matrix3d& Triad);

	void RotatationMatrix2Quarternion(Eigen::Matrix3d a, Eigen::Vector4d& Quarternion);

	void Prediction(Eigen::Vector3d Gyro, long long T);

	double Comparator(Eigen::Vector4d q1, Eigen::Vector4d q2);

	void Correction();

	Eigen::Matrix3d InitialTriad;

	Eigen::Vector3d Mag_0;
	Eigen::Vector3d Acc_0;
	Eigen::Vector3d Mag_1;
	Eigen::Vector3d Acc_1;
	Eigen::Vector3d Gyro_w;
	

	std::fstream fout;
	std::string file_name = "D:/GITProjects/Kalman Filtering Server/PoseEstimationKF/Sensor_CSV/KalmanFilter.txt";
	bool first_update = true;

	Eigen::Matrix<double, 4, 1> Quarternion_Gyro_pure;


};


