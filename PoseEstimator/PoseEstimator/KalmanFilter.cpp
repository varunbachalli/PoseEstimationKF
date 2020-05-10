#include "KalmanFilter.h"
#include <thread>
#include <chrono>

static const double M_PI = 3.14159265358979323846;
KalmanFilter::KalmanFilter() {}


KalmanFilter::KalmanFilter(std::map<std::string, std::array<double, 3>> Values, long long timestamp)
{
	set_mag_0(Values.find("mag0")->second);
	set_acc_0(Values.find("acc0")->second);
	set_mag_sig(Values.find("mag_sig")->second);
	set_acc_sig(Values.find("acc_sig")->second);
	set_gyro_drift_0(Values.find("gyro_drift_0")->second);
	set_gyro_drift_sig(Values.find("gyro_drift_sig")->second);
	compute_initial_params();
	std::cout << "Init Kalman" << std::endl;
	previousT = timestamp;
}

void KalmanFilter::set_mag_0(std::array<double, 3> mag) { Mag_0 << mag[0], mag[1], mag[2]; std::string text = "mag_0 : " + std::to_string(mag[0]) + ","+ std::to_string(mag[1]) + ","+ std::to_string(mag[2]); WriteTextFile(text); }
void KalmanFilter::set_acc_0(std::array<double, 3> acc) { Acc_0 << acc[0], acc[1], acc[2]; std::string text = "acc_0 : " + std::to_string(acc[0]) + "," + std::to_string(acc[1]) + "," + std::to_string(acc[2]); WriteTextFile(text);}
void KalmanFilter::set_mag_sig(std::array<double, 3> mag_sig) { R_Sigma[0] = 10 * mag_sig[0]; R_Sigma[1] = 10 * mag_sig[1]; R_Sigma[2] = 10 * mag_sig[2]; }
void KalmanFilter::set_acc_sig(std::array<double, 3> acc_sig) { R_Sigma[3] = 10 * acc_sig[0]; R_Sigma[4] = 10 * acc_sig[1]; R_Sigma[5] = 10 * acc_sig[2]; }
void KalmanFilter::set_gyro_drift_0(std::array<double, 3> gyro_drift) { Gyro_drift_0 << gyro_drift[0], gyro_drift[1], gyro_drift[2]; }
void KalmanFilter::set_gyro_drift_sig(std::array<double, 3> gyro_sig) { Gyro_Sigma << gyro_sig[0], gyro_sig[1], gyro_sig[2]; }

Eigen::Matrix3d KalmanFilter::WahbaSVDRotation(Eigen::Vector3d Mag, Eigen::Vector3d Acc, double k_acc, double k_mag)
{
	Eigen::Matrix3d B_acc = k_acc * Acc * Acc_0.transpose();
	Eigen::Matrix3d B_mag = k_mag * Mag * Mag_0.transpose();
	Eigen::Matrix3d B_total = B_acc + B_mag;
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(B_total, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d VT = svd.matrixV().transpose();
	double Udet = U.determinant();
	double Vdet = VT.determinant();
	Eigen::Matrix3d M;
	M << 1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, Udet* Vdet;

	Eigen::Matrix3d RotationMatrix = U * M * VT;
	return RotationMatrix;
}

void KalmanFilter::compute_initial_params()
{
	Quarternion_Gyro_pure << 1.0, 0.0, 0.0, 0.0;
	X_k << 1.0, 0.0, 0.0, 0.0;

	std::string text = "q_gyro : 1.0, 0.0, 0.0, 0.0";
	WriteTextFile(text);

	text = "X_k : 1.0, 0.0, 0.0, 0.0";
	WriteTextFile(text);

	text = "Wahba_quart : 1.0, 0.0, 0.0, 0.0";
	WriteTextFile(text);

	Q_k = Eigen::MatrixXd::Identity(3, 3);
	Q_k = Q_k * 0.01;
	
	R_k = Eigen::Matrix4d::Identity(4, 4);
	R_k = R_k * 0.1;

	P_k = Eigen::MatrixXd::Identity(4, 4);
}

void KalmanFilter::ComputeTriad(Eigen::Vector3d s, Eigen::Vector3d m, Eigen::Matrix3d& Triad) 
{
	Eigen::Vector3d t1 = s / s.norm();
	Eigen::Vector3d t2 = s.cross(m);
	t2 = t2 / t2.norm();
	Eigen::Vector3d t3 = t2.cross(t1);
	Triad(0, 0) = t1(0);
	Triad(1, 0) = t1(1);
	Triad(2, 0) = t1(2);

	Triad(0, 1) = t2(0);
	Triad(1, 1) = t2(1);
	Triad(2, 1) = t2(2);

	Triad(0, 2) = t3(0);
	Triad(1, 2) = t3(1);
	Triad(2, 2) = t3(2);
}



void KalmanFilter::RotatationMatrix2Quarternion(Eigen::Matrix3d M, Eigen::Vector4d& Quart) // change this
{
	double tr1 = 1.0 + M(0, 0) - M(1, 1) - M(2, 2);
	double tr2 = 1.0 - M(0, 0) + M(1, 1) - M(2, 2);
	double tr3 = 1.0 - M(0, 0) - M(1, 1) + M(2, 2);

	if ((tr1 > tr2) && (tr1 > tr3))
	{
		double S = sqrt(tr1) * 2; // S = 4 * qx
		double qw = (M(2, 1) - M(1, 2)) / S;
		double qx = 0.25 * S;
		double qy = (M(0, 1) + M(1, 0)) / S;
		double qz = (M(0, 2) + M(2, 0)) / S;
		Quart << qw, qx, qy, qz;
	}
	else if ((tr2 > tr1) && (tr2 > tr3))
	{
		double S = sqrt(tr2) * 2; // S = 4 * qy
		double qw = (M(0, 2) - M(2, 0)) / S;
		double qx = (M(0, 1) + M(1, 0)) / S;
		double qy = 0.25 * S;
		double qz = (M(1, 2) + M(2, 1)) / S;
		Quart << qw, qx, qy, qz;
	}
	else
	{
		double S = sqrt(tr3) * 2; // S = 4 * qz
		double qw = (M(1, 0) - M(0, 1)) / S;
		double qx = (M(0, 2) + M(2, 0)) / S;
		double qy = (M(1, 2) + M(2, 1)) / S;
		double qz = 0.25 * S;
		Quart << qw, qx, qy, qz;
	}
}

void KalmanFilter::Prediction(Eigen::Vector3d Gyro, long long T) // Time is in Nano Seconds
{
	if (first_update)
	{
		std::string text = "T : " + std::to_string(previousT);
		WriteTextFile(text);
		first_update = false;
	}
	Eigen::Matrix<double, 4, 4> Jacobian_A;
	GetJacobian_A(Gyro, Jacobian_A);
	Eigen::Matrix<double, 4, 3> Jacobian_B;
	GetJacobian_B(X_k, Jacobian_B);
	P_k = Jacobian_A * P_k * Jacobian_A.transpose() + Jacobian_B*Q_k*Jacobian_B.transpose();
	Eigen::Vector4d PrevX_k = X_k;
	RungeKuttaEval(X_k, T, Gyro);
	RungeKuttaEval(Quarternion_Gyro_pure, T, Gyro);
	std::string text = "T : " + std::to_string(T);
	WriteTextFile(text);
	text = "q_gyro : " + std::to_string(Quarternion_Gyro_pure(0)) + "," + std::to_string(Quarternion_Gyro_pure(1)) + "," + std::to_string(Quarternion_Gyro_pure(2)) + "," + std::to_string(Quarternion_Gyro_pure(3));
	WriteTextFile(text);
	Eigen::Vector3d Change_in_angle;
	getRPY_1(X_k - PrevX_k, Change_in_angle);
	S_k = P_k + R_k;
	K_k = P_k * S_k.inverse();
	z_k = X_k;
	previousT = T;
}

void KalmanFilter::Correction()
{
	//Eigen::Matrix3d RotationMatrix = WahbaSVDRotation(Mag_1, Acc_1, abs(Acc_1[2]), 1 - (abs(Acc_1[2]))).transpose();
	Eigen::Matrix3d RotationMatrix = WahbaSVDRotation(Mag_1, Acc_1, 0.5, 0.5);
	RotationMatrix.transposeInPlace();
	Eigen::Vector4d Y_k;
	RotatationMatrix2Quarternion(RotationMatrix, Y_k);
	double comparator = Y_k.dot(z_k);
	if (comparator <= 1.2 && comparator >= 0.9)
	{
		X_k = X_k + (K_k * (Y_k - z_k));
	}
	else if(comparator >= -1.2 && comparator <= -0.9)
	{
		X_k = X_k + (K_k * (-Y_k - z_k));
	}
	
	X_k = X_k / X_k.norm();
	P_k = P_k - (K_k * P_k);
	std::string text = "X_k : " + std::to_string(X_k(0)) + "," + std::to_string(X_k(1)) + "," + std::to_string(X_k(2)) + "," + std::to_string(X_k(3));
	WriteTextFile(text);
	text = "Wahba_quart : " + std::to_string(Y_k(0)) + "," + std::to_string(Y_k(1)) + "," + std::to_string(Y_k(2)) + "," + std::to_string(Y_k(3));
	WriteTextFile(text);
}

void KalmanFilter::getXk_1(double* quart)
{
	quart[0] = X_k(0);
	quart[1] = X_k(1);
	quart[2] = X_k(2);
	quart[3] = X_k(3);
}




void KalmanFilter::getRPY_1(Eigen::Vector4d q, Eigen::Vector3d& angles)
{

	double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
	double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
	angles(0) = std::atan2(sinr_cosp, cosr_cosp);
	double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
	if (std::abs(sinp) >= 1)
		angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		angles(1) = std::asin(sinp);
	double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
	double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
	angles(2) = std::atan2(siny_cosp, cosy_cosp);

	angles = angles * 180.0 / M_PI;
}
void KalmanFilter::getRPY(double* angles)
{
	Eigen::Vector4d q;
	q << X_k(0), X_k(1), X_k(2), X_k(3);


	double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
	double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
	angles[0] = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
	if (std::abs(sinp) >= 1)
		angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		angles[1] = std::asin(sinp);
	double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
	double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
	angles[2] = std::atan2(siny_cosp, cosy_cosp);

	angles[0] = angles[0] * 180.0 / M_PI;
	angles[1] = angles[1] * 180.0 / M_PI;
	angles[2] = angles[2] * 180.0 / M_PI;
}

void KalmanFilter::UpdateLatestPreviousTime(long long Time)
{
	previousT = Time;
}

void KalmanFilter::GetJacobian_A(Eigen::Vector3d U0, Eigen::Matrix4d& Jacobian)
{
	/*Jacobian = Eigen::Matrix4d::Zero();*/
	double w1, w2, w0;
	w0 = U0(0);
	w1 = U0(1);
	w2 = U0(2);

	Jacobian << 0.0, -w0, w1, w2,
				w0, 0.0, w2, -w1,
				w1, -w2, 0.0, w0,
				w2, w1, -w0, 0.0;
	Jacobian = Jacobian / 2;
}
void KalmanFilter::GetJacobian_B(Eigen::Vector4d q, Eigen::Matrix<double, 4,3>& Jacobian)
{
	Jacobian << -q(1), -q(2), -q(3),
				 q(0), q(3), -q(2),
				 -q(3), q(0), q(1),
				 q(2), -q(1), q(0);

	Jacobian = Jacobian / 2;

}
void KalmanFilter::SetAngularVelocity(double w_x, double w_y, double w_z, long long Time)
{
	Gyro_w(0) = -w_x;
	Gyro_w(1) = -w_y;
	Gyro_w(2) = -w_z;
	std::string text = "gyro : " + std::to_string(Gyro_w(0)) + "," + std::to_string(Gyro_w(1)) + "," + std::to_string(Gyro_w(2));
	WriteTextFile(text);
	Prediction(Gyro_w, Time);
}

void KalmanFilter::SetMagnetometerMeasurements(double m_x, double m_y, double m_z)
{
	Mag_1(0) = m_x;
	Mag_1(1) = m_y;
	Mag_1(2) = m_z;
	std::string text = "Mag_1 : " + std::to_string(Mag_1(0)) + "," + std::to_string(Mag_1(1)) + "," + std::to_string(Mag_1(2));
	WriteTextFile(text);

}

void KalmanFilter::SetAccelerometerMeasurements(double a_x, double a_y, double a_z)
{
	Acc_1(0) = a_x;
	Acc_1(1) = a_y;
	Acc_1(2) = a_z;
	std::string text = "Acc_1 : " + std::to_string(Acc_1(0)) + "," + std::to_string(Acc_1(1)) + "," + std::to_string(Acc_1(2));
	WriteTextFile(text);
	Correction();
}


void KalmanFilter::RungeKuttaEval(Eigen::Vector4d& q0, long long T, Eigen::Vector3d U0) // also use previous T in Nano
{

	double dT = (T - previousT) * pow(10, -9);
	double w1, w2, w0;
	w0 = U0(0);
	w1 = U0(1);
	w2 = U0(2);

	Eigen::Matrix4d W;
	W << 0.0, -w0, -w1, -w2,
		w0, 0.0, w2, -w1,
		w1, -w2, 0.0, w0,
		w2, w1, -w0, 0.0;
	W = W / 2;

	Eigen::Vector4d k1, k2, k3, k4;
	k1 = W * q0;
	k2 = W * (q0 + (dT * k1 / 2));
	k3 = W * (q0 + (dT * k2 / 2));
	k4 = W * (q0 + dT * k3);

	q0 = q0 + (dT / 6) * (k1 + 2 * k2 + 2 * k3 + k4);

	double Normq1 = q0.norm();
	q0 = q0 / Normq1;
}


void KalmanFilter::WriteTextFile(std::string text)
{
	fout.open(file_name, std::ios::out | std::ios::app);
	fout << text << std::endl;
	fout.close();
}

// Things to transfer : 1. Acc_0 , Mag_0, T_0
// Acc, Gyro, Mag, X_k, Triad_quart, Gyro_quart, T