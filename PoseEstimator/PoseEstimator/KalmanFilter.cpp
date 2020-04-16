#include "KalmanFilter.h"
#include <conio.h>

/*
goal : 
set sensor values along 
	with time stamp.
return 
	quarternion 
return 
	Roll pitch yaw
*/

KalmanFilter::KalmanFilter(std::map<std::string, std::array<double, 3>> Values)
{
	set_mag_0(Values.find("mag0")->second);
	set_acc_0(Values.find("acc0")->second);
	set_mag_sig(Values.find("mag_sig")->second);
	set_acc_sig(Values.find("acc_sig")->second);
	set_gyro_drift_0(Values.find("gyro_drift_0")->second);
	set_gyro_drift_sig(Values.find("gyro_drift_sig")->second);
	compute_initial_params();
}

void KalmanFilter::set_mag_0(std::array<double, 3> mag) {Mag_0 << mag[0], mag[1], mag[2];}
void KalmanFilter::set_acc_0(std::array<double, 3> acc) { Acc_0 << acc[0], acc[1], acc[2];}
void KalmanFilter::set_mag_sig(std::array<double, 3> mag_sig) { R_Sigma[0] = mag_sig[0]; R_Sigma[1] = mag_sig[1]; R_Sigma[2] = mag_sig[2]; }
void KalmanFilter::set_acc_sig(std::array<double, 3> acc_sig) { R_Sigma[3] = acc_sig[0]; R_Sigma[4] = acc_sig[1]; R_Sigma[5] = acc_sig[2]; }
void KalmanFilter::set_gyro_drift_0(std::array<double, 3> gyro_drift) { Gyro_drift_0 << gyro_drift[0], gyro_drift[1], gyro_drift[2];}
void KalmanFilter::set_gyro_drift_sig(std::array<double, 3> gyro_sig) { Gyro_Sigma << gyro_sig[0], gyro_sig[1], gyro_sig[2];}

void KalmanFilter::compute_initial_params() 
{
	Xk << 1, 0, 0, 0, Gyro_drift_0(0), Gyro_drift_0(1), Gyro_drift_0(2);

	Q_k = Eigen::MatrixXd::Identity(7, 7);
    double temp[] = { 0.1, 0.1, 0.1, 0.1, Gyro_Sigma(0), Gyro_Sigma(1), Gyro_Sigma(2) };
    for (int i = 0; i < 7; ++i)
    {
        Q_k(i, i) = temp[i];
    }

    double MaxElement =  *std::max_element(R_Sigma.begin(), R_Sigma.end());
 
    R_k.topLeftCorner(4, 4).setIdentity();
    R_k.topRightCorner(4, 3).setZero();
    R_k.bottomLeftCorner(3, 4).setZero();
    R_k.bottomRightCorner(3, 3).setZero();
    R_k = R_k * MaxElement;

    InitialTriad = ComputeTriad(Acc_0, Mag_0);
	
	/*
		q_0 = [1,0,0,0](done)
		q_variance_0 = [0.1, 0.1, 0.1, 0.1](done)
		compute X0 with q_0 and gyro_drift (done)
		compute t0 with mag0 and acc0 (done)
		compute Q with gyro sigma and q_variance_0(done)
		compute R with mag_sigma and acc_sigma [max of the 2 should be a good estimate](done)
	*/
}

Eigen::Matrix3d KalmanFilter::ComputeTriad(Eigen::Vector3d s, Eigen::Vector3d m)
{
    Eigen::Matrix3d Triad;
    Eigen::Vector3d t1 = s/s.norm();
    Eigen::Vector3d t2 = s.cross(m);
    t2 = t2 / t2.norm();
    Eigen::Vector3d t3 = t2.cross(t1);
    t3 = t3 / t3.norm();
    Triad << t1, t2, t3;
    return Triad;
}

Eigen::Vector4d KalmanFilter::ConvertRotatationMatrix2Quarternion(Eigen::Matrix3d a)
{
    Eigen::Vector4d Quarternion;
    double trace = a(0,0) + a(1,1) + a(2,2); 
    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        Quarternion(0) = 0.25 / s;
        Quarternion(1) = (a(2,1) - a(1,2)) * s;
        Quarternion(2) = (a(0,2) - a(2,0)) * s;
        Quarternion(3) = (a(1,0) - a(0,1)) * s;
    }
    else {
        if (a(0, 0) > a(1, 1) && a(0, 0) > a(2, 2)) {
            double s = 2.0 * sqrt(1.0 + a(0, 0) - a(1, 1) - a(2, 2));
            Quarternion(0) = (a(2,1) - a(1,2)) / s;
            Quarternion(1) = 0.25 * s;
            Quarternion(2) = (a(0,1) + a(1,0)) / s;
            Quarternion(3) = (a(0,2) + a(2,0)) / s;
        }
        else if (a(1, 1) > a(2, 2)) {
            double s = 2.0 * sqrt(1.0 + a(1, 1) - a(0, 0) - a(2, 2));
            Quarternion(0) = (a(0,2) - a(2,0)) / s;
            Quarternion(1) = (a(0,1) + a(1,0)) / s;
            Quarternion(2) = 0.25 * s;
            Quarternion(3) = (a(1,2) + a(2,1)) / s;
        }
        else {
            double s = 2.0 * sqrt(1.0 + a(2, 2) - a(0, 0) - a(1, 1));
            Quarternion(0) = (a(1,0) - a(0,1)) / s;
            Quarternion(1) = (a(0,2) + a(2,0)) / s;
            Quarternion(2) = (a(1,2) + a(2,1)) / s;
            Quarternion(3) = 0.25 * s;
        }
    }
    return Quarternion;
}

void KalmanFilter::SetAngularVelocity(double w_x, double w_y, double w_z, long Time)
{
	
}

void KalmanFilter::SetMagnetometerMeasurements(double m_x, double m_y, double m_z)
{

}

void KalmanFilter::SetAccelerometerMeasurements(double a_x, double a_y, double a_z)
{

}





Eigen::Matrix<double, 7,1> KalmanFilter::RungeKuttaEval(Eigen::Matrix<double, 7, 1> X_0, double T, Eigen::Matrix3d w)
{
    return Eigen::Matrix<double, 7, 1>();
}

int main()
{
    Eigen::Matrix<double, 7, 7> R_k;
    R_k.topLeftCorner(4, 4).setIdentity();
    R_k.topRightCorner(4, 3).setZero();
    R_k.bottomLeftCorner(3, 4).setZero();
    R_k.bottomRightCorner(3, 3).setZero();
    std::cout << R_k << std::endl;
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Identity(7, 7);

    double temp[] = { 0.1, 0.1, 0.1, 0.1, 10.0, 10.0, 10.0 };
    for (int i = 0; i < 7; ++i)
    {
        Q_k(i, i) = temp[i];
    }
    std::cout << Q_k << std::endl;
    _getch();
    return 0;
}

/*

// Prediction

setw_k
// init Pk

setQk
setRk


getJacobian()
getXk1_k = evaluate state space without noise

getPk1_k = Evaluate Jacobian and Qk equation.


getSk1 Pk_1, Hk_1 Rk1 equation

getK1 = Pk_1 Hk_1 Sk1 Inverse equation


// Correction
getXk1_k1

setPk1_k1

*/