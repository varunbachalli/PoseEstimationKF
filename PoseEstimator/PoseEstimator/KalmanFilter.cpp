#include "KalmanFilter.h"


/*
goal : 
set sensor values along 
	with time stamp.
return 
	quarternion 
return 
	Roll pitch yaw
*/

static const double M_PI = 3.14159265358979323846;

KalmanFilter::KalmanFilter(std::map<std::string, std::array<double, 3>> Values, long timestamp)
{
	set_mag_0(Values.find("mag0")->second);
	set_acc_0(Values.find("acc0")->second);
	set_mag_sig(Values.find("mag_sig")->second);
	set_acc_sig(Values.find("acc_sig")->second);
	set_gyro_drift_0(Values.find("gyro_drift_0")->second);
	set_gyro_drift_sig(Values.find("gyro_drift_sig")->second);
	compute_initial_params();
    previousT = timestamp;
}

void KalmanFilter::set_mag_0(std::array<double, 3> mag) {Mag_0 << mag[0], mag[1], mag[2];}
void KalmanFilter::set_acc_0(std::array<double, 3> acc) { Acc_0 << acc[0], acc[1], acc[2];}
void KalmanFilter::set_mag_sig(std::array<double, 3> mag_sig) { R_Sigma[0] = mag_sig[0]; R_Sigma[1] = mag_sig[1]; R_Sigma[2] = mag_sig[2]; }
void KalmanFilter::set_acc_sig(std::array<double, 3> acc_sig) { R_Sigma[3] = acc_sig[0]; R_Sigma[4] = acc_sig[1]; R_Sigma[5] = acc_sig[2]; }
void KalmanFilter::set_gyro_drift_0(std::array<double, 3> gyro_drift) { Gyro_drift_0 << gyro_drift[0], gyro_drift[1], gyro_drift[2];}
void KalmanFilter::set_gyro_drift_sig(std::array<double, 3> gyro_sig) { Gyro_Sigma << gyro_sig[0], gyro_sig[1], gyro_sig[2];}

void KalmanFilter::compute_initial_params() 
{
	X_k << 1, 0, 0, 0, Gyro_drift_0(0), Gyro_drift_0(1), Gyro_drift_0(2);

	Q_k = Eigen::MatrixXd::Identity(7, 7);
    double temp[] = { 0.1, 0.1, 0.1, 0.1, Gyro_Sigma(0), Gyro_Sigma(1), Gyro_Sigma(2) };
    for (int i = 0; i < 7; ++i)
    {
        Q_k(i, i) = temp[i];
    }

    double MaxElement =  *std::max_element(R_Sigma.begin(), R_Sigma.end());
   
    R_k = Eigen::Matrix4d::Identity(4, 4);
    R_k = R_k * MaxElement;
    InitialTriad = ComputeTriad(Acc_0, Mag_0);

    P_k = Eigen::MatrixXd::Identity(7, 7);
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

Eigen::Vector4d KalmanFilter::RotatationMatrix2Quarternion(Eigen::Matrix3d a)
{
    Eigen::Vector4d Quarternion;
    double trace = a(0,0) + a(1,1) + a(2,2); 
    if (trace > 0) 
    {
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

void KalmanFilter::Prediction(Eigen::Vector3d Gyro, long T) // Time is in Nano Seconds
{
    Eigen::Matrix<double, 7, 7> Jacobian = GetJacobian(X_k, Gyro, T);
    P_k = Jacobian * P_k * Jacobian.transpose() + Q_k;
    X_k = RungeKuttaEval(X_k, T, Gyro);
    Eigen::Matrix<double, 4, 7> H;
    H << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0;
    S_k = H * P_k * H.transpose() + R_k;
    K_k = P_k * H.transpose() * S_k.inverse();
    z_k = H * X_k;
}

void KalmanFilter::Correction()
{
    Eigen::Matrix3d Triad_new;
    Triad_new = ComputeTriad(Acc_1, Mag_1);
    Eigen::Matrix3d RotationMatrix = Triad_new * InitialTriad.transpose();
    Eigen::Vector4d Quart = RotatationMatrix2Quarternion(RotationMatrix);
    X_k = X_k + K_k * (z_k - Quart);
    P_k = P_k - K_k * S_k * K_k.transpose();
}

Eigen::Vector4d KalmanFilter::getXk_1()
{
    Eigen::Vector4d quart;
    quart << X_k(0), X_k(1), X_k(2), X_k(3);
    return quart;
}


Eigen::Vector3d KalmanFilter::getRPY()
{
    Eigen::Vector3d angles;
    Eigen::Vector4d q;
    q << X_k(0), X_k(1), X_k(2), X_k(3);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}




Eigen::Matrix<double, 7, 7> KalmanFilter::GetJacobian(Eigen::Matrix<double, 7, 1> X_0, Eigen::Vector3d U0, long T)
{
    Eigen::Matrix<double, 7, 7> Jacobian;
    double q0, q1, q2, q3 , b1, b2, b3;
    q0 = X_0(0, 0);
    q1 = X_0(1, 0);
    q2 = X_0(2, 0);
    q3 = X_0(3, 0);
    b1 = X_0(4, 0);
    b2 = X_0(5, 0);
    b3 = X_0(6, 0);


    double w1, w2, w3;
    w1 = U0(0);
    w2 = U0(1);
    w3 = U0(2);
    Jacobian << 1,          -(w1-b1),        -(w2-b2),       -(w3-b3),       q1,     q2,     q3,
                (w1-b1),        1 ,           (w3-b3),       -(w2-b2),      -q0,     q3,    -q2,
                (w2-b2),    -(w3-b3),           1,            (w1-b1),      -q3,    -q0,     q1,
                (w3-b3),     (w2-b2),        -(w1-b1),           1,          q2,    -q1,    -q0,
                0,              0,              0,              0,            1,      0,      0,
                0,              0,              0,              0,            0,      1,      0,
                0,              0,              0,              0,            0,      0,      1;
    Jacobian = ((T - previousT)*pow(10,-9)) * Jacobian / 2;
    return Jacobian;
}
void KalmanFilter::SetAngularVelocity(double w_x, double w_y, double w_z, long Time)
{
    Eigen::Vector3d Gyro_w;
    Gyro_w << w_x, w_y, w_z;
    Prediction(Gyro_w,Time);
}

void KalmanFilter::SetMagnetometerMeasurements(double m_x, double m_y, double m_z)
{
    Mag_1 << m_x, m_y, m_z;
}

void KalmanFilter::SetAccelerometerMeasurements(double a_x, double a_y, double a_z)
{
    Acc_1 << a_x, a_y, a_z;
    Correction();
}


Eigen::Matrix<double, 7, 1> KalmanFilter::RungeKuttaEval(Eigen::Matrix<double, 7, 1> X_0, double T, Eigen::Vector3d U0) // also use previous T in Nano
{

    double T_ = (T - previousT) * pow(10, -9);

    double q0, q1, q2, q3, b1, b2, b3;
    q0 = X_0(0, 0);
    q1 = X_0(1, 0);
    q2 = X_0(2, 0);
    q3 = X_0(3, 0);
    b1 = X_0(4, 0);
    b2 = X_0(5, 0);
    b3 = X_0(6, 0);

    double w1, w2, w3;
    w1 = U0(0);
    w2 = U0(1);
    w3 = U0(2);

    Eigen::Matrix<double, 4, 1> f_x;
    f_x << q0 - (w1 - b1) * q1 - (w2 - b2) * q2 - (w3 - b3) * q3,
           (w1 - b1)* q0 + q1 + (w3 - b3) * q2 - (w2 - b2) * q3,
           (w2 - b2)* q0 - (w3 - b3) * q1 + q2 + (w1 - b1) * q3,
           (w3 - b3)* q0 + (w2 - b2) * q1 - (w1 - b1) * q2 + q3;
    f_x = f_x / 2;

    Eigen::Matrix<double, 4, 1> k1;
    k1 = f_x;
    // k1 = f(tn,yn)
    Eigen::Matrix<double, 4, 1> k2;
    q0 = X_0(0, 0) + T_ / 2 * k1(0);
    q1 = X_0(1, 0) + T_ / 2 * k1(1);
    q2 = X_0(2, 0) + T_ / 2 * k1(2);
    q3 = X_0(3, 0) + T_ / 2 * k1(3);
    // yn = yn + h*k1/2
    k2 << q0 - (w1 - b1) * q1 - (w2 - b2) * q2 - (w3 - b3) * q3,
        (w1 - b1)* q0 + q1 + (w3 - b3) * q2 - (w2 - b2) * q3,
        (w2 - b2)* q0 - (w3 - b3) * q1 + q2 + (w1 - b1) * q3,
        (w3 - b3)* q0 + (w2 - b2) * q1 - (w1 - b1) * q2 + q3;
    k2 = k2 / 2;
    // k2 = f(tn,yn)
    q0 = X_0(0, 0) +  T_ / 2 * k2(0);
    q1 = X_0(1, 0) +  T_ / 2 * k2(1);
    q2 = X_0(2, 0) +  T_ / 2 * k2(2);
    q3 = X_0(3, 0) +  T_ / 2 * k2(3);
    // yn = yn + h*k2/2
    Eigen::Matrix<double, 4, 1> k3;
    k3 << q0 - (w1 - b1) * q1 - (w2 - b2) * q2 - (w3 - b3) * q3,
        (w1 - b1)* q0 + q1 + (w3 - b3) * q2 - (w2 - b2) * q3,
        (w2 - b2)* q0 - (w3 - b3) * q1 + q2 + (w1 - b1) * q3,
        (w3 - b3)* q0 + (w2 - b2) * q1 - (w1 - b1) * q2 + q3;
    k3 = k3 / 2;
    // k2 = f(tn,yn)
    q0 = X_0(0, 0) + T_ * k3(0);
    q1 = X_0(1, 0) + T_ * k3(1);
    q2 = X_0(2, 0) + T_ * k3(2);
    q3 = X_0(3, 0) + T_ * k3(3);
    // yn = yn + h*k3
    Eigen::Matrix<double, 4, 1> k4;

    k4 << q0 - (w1 - b1) * q1 - (w2 - b2) * q2 - (w3 - b3) * q3,
        (w1 - b1)* q0 + q1 + (w3 - b3) * q2 - (w2 - b2) * q3,
        (w2 - b2)* q0 - (w3 - b3) * q1 + q2 + (w1 - b1) * q3,
        (w3 - b3)* q0 + (w2 - b2) * q1 - (w1 - b1) * q2 + q3;
    k4 = k4 / 2;
    // k2 = f(tn,yn)
    Eigen::Matrix<double, 4, 1> q_k1;
    Eigen::Matrix<double, 4, 1> q_k;
    q_k << X_0(0, 0),
           X_0(1, 0),
           X_0(2, 0),
           X_0(3, 0);

    q_k1 = q_k + T_ / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    // yn+1 = yn + 1/6 * h * (k1 + 2k2 + 2k3 + k4)
    X_0(0 ,0) = q_k1(0, 0);
    X_0(1, 0) = q_k1(1, 0);
    X_0(2, 0) = q_k1(2, 0);
    X_0(3, 0) = q_k1(3, 0);
    return X_0;

}

int main()
{
    Eigen::Matrix<double, 7, 1> X_0;
    X_0 << 1, 0, 0, 0, 0,0,0;
    double q0, q1, q2, q3, b1, b2, b3;
    q0 = X_0(0, 0);
    q1 = X_0(1, 0);
    q2 = X_0(2, 0);
    q3 = X_0(3, 0);
    b1 = X_0(4, 0);
    b2 = X_0(5, 0);
    b3 = X_0(6, 0);

    static const double pi = 3.14159265358979323846;

    double w1, w2, w3;
    w1 = pi/2;
    w2 = pi/2;
    w3 = 0.0;

    Eigen::Matrix<double, 7, 1> f_x;
    f_x <<  q0 - (w1 - b1) * q1 - (w2 - b2) * q2 - (w3 - b3) * q3,
            (w1 - b1)* q0 + q1 + (w3 - b3) * q2 - (w2 - b2) * q3,
            (w2 - b2)* q0 - (w3 - b3) * q1 + q2 + (w1 - b1) * q3,
            (w3 - b3)* q0 + (w2 - b2) * q1 - (w1 - b1) * q2 + q3,
            b1,
            b2,
            b3;

    Eigen::Matrix<double, 7, 1> kk;
    kk = f_x + f_x;

    std::cout << f_x;

    std::cout << "\n" << kk << std::endl;
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