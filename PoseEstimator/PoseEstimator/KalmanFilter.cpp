#include "KalmanFilter.h"
#include <thread>
#include <chrono>

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
KalmanFilter::KalmanFilter(){ InitCSVFile(); }


KalmanFilter::KalmanFilter(std::map<std::string, std::array<double, 3>> Values, long long timestamp)
{
	set_mag_0(Values.find("mag0")->second);
	set_acc_0(Values.find("acc0")->second);
	set_mag_sig(Values.find("mag_sig")->second);
	set_acc_sig(Values.find("acc_sig")->second);
	set_gyro_drift_0(Values.find("gyro_drift_0")->second);
	set_gyro_drift_sig(Values.find("gyro_drift_sig")->second);
	compute_initial_params();
    previousT = timestamp;
    std::cout << "Kalman Filter initialized " << std::endl;
}

void KalmanFilter::set_mag_0(std::array<double, 3> mag) { Mag_0 << mag[0], mag[1], mag[2]; memcpy(&data.mag, &mag, sizeof(std::array<double, 3>)); }
void KalmanFilter::set_acc_0(std::array<double, 3> acc) { Acc_0 << acc[0], acc[1], acc[2]; memcpy(&data.acc, &acc, sizeof(std::array<double, 3>)); }
void KalmanFilter::set_mag_sig(std::array<double, 3> mag_sig)  { R_Sigma[0] = 10*mag_sig[0]; R_Sigma[1] = 10 * mag_sig[1]; R_Sigma[2] = 10 * mag_sig[2]; }
void KalmanFilter::set_acc_sig(std::array<double, 3> acc_sig) { R_Sigma[3] = 10 * acc_sig[0]; R_Sigma[4] = 10 * acc_sig[1]; R_Sigma[5] = 10 * acc_sig[2]; }
void KalmanFilter::set_gyro_drift_0(std::array<double, 3> gyro_drift) { Gyro_drift_0 << gyro_drift[0], gyro_drift[1], gyro_drift[2];}
void KalmanFilter::set_gyro_drift_sig(std::array<double, 3> gyro_sig) { Gyro_Sigma << gyro_sig[0], gyro_sig[1], gyro_sig[2];}

void KalmanFilter::compute_initial_params() 
{  
    Quarternion_Gyro_pure << 1.0, 0.0, 0.0, 0.0;
    X_k << 1.0, 0.0, 0.0, 0.0;
	Q_k = Eigen::MatrixXd::Identity(4, 4);
    //double temp[] = { 0.1, 0.1,  0.1,  0.1 };  
    /*for (int i = 0; i < 4; ++i)
    {
        Q_k(i, i) = temp[i];
    }*/

    double MaxElement = *std::max_element(R_Sigma.begin(), R_Sigma.end());
   
    R_k = Eigen::Matrix4d::Identity(4, 4);
    R_k = R_k * 0.1;
    ComputeTriad(Mag_0, Acc_0, InitialTriad);
    //std::cout << InitialTriad << std::endl;
    P_k = Eigen::MatrixXd::Identity(4, 4);
    //P_k = 1.0 * P_k;

    data.gyr = std::array<double, 3>{0.0,0.0,0.0};
    data.quarternion_measurement = std::array<double, 4>{1.0, 0.0, 0.0,0.0};
    data.quarternion_predict = std::array<double, 4>{1.0, 0.0, 0.0,0.0};
}

void KalmanFilter::ComputeTriad(Eigen::Vector3d s, Eigen::Vector3d m, Eigen::Matrix3d& Triad)
{
    Eigen::Vector3d t1 = s/s.norm();
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

void KalmanFilter::RotatationMatrix2Quarternion(Eigen::Matrix3d a, Eigen::Vector4d& Quarternion)
{
    
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
}

void KalmanFilter::Prediction(Eigen::Vector3d Gyro, long long T) // Time is in Nano Seconds
{
    Eigen::Matrix<double, 4, 4> Jacobian;
    GetJacobian(Gyro, Jacobian);
    //std::cout << "1\n";
    P_k = Jacobian * P_k * Jacobian.transpose() + Q_k;
    //std::cout << "2\n";
    Eigen::Vector4d PrevX_k = X_k;
    //std::cout << "3\n";
    RungeKuttaEval(X_k, T, Gyro);
    RungeKuttaEval(Quarternion_Gyro_pure, T, Gyro);
    data.quarternion_Gyro_pure[0] = Quarternion_Gyro_pure(0);
    data.quarternion_Gyro_pure[1] = Quarternion_Gyro_pure(1);
    data.quarternion_Gyro_pure[2] = Quarternion_Gyro_pure(2);
    data.quarternion_Gyro_pure[3] = Quarternion_Gyro_pure(3);
    //std::cout << "4\n";
    Eigen::Vector3d Change_in_angle;
    getRPY_1(X_k - PrevX_k , Change_in_angle);
    //std::cout << "5\n";
    /*printf("Change_in_angle is [%f,%f,%f]\n", Change_in_angle(0), Change_in_angle(1), Change_in_angle(2));*/
    /*std::cout << "change in angle is " << Change_in_angle(2) << std::endl;*/

    Eigen::Matrix4d H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    S_k = H * P_k * H.transpose() + R_k;
    //std::cout << "6\n";
    K_k = P_k * H.transpose() * S_k.inverse();
    //std::cout << "7\n";
    z_k = H * X_k;
    //std::cout << "8\n";
    data.quarternion_predict[0] = z_k(0);
    data.quarternion_predict[1] = z_k(1);
    data.quarternion_predict[2] = z_k(2);
    data.quarternion_predict[3] = z_k(3);
    //std::cout << "Prediction Step"<< std::endl;
    //if (numberofPrints < max && numberofPrints > min)
    //{
    //    /*std::cout << "\n K_k is \n" << K_k << std::endl;*/
    //    printf("predicted Quarternion is [%f,%f,%f,%f]\n", z_k(0), z_k(1), z_k(2), z_k(3));
    //    std::cout << "Predicted Yaw" << getRPY_1(X_k)(2) << std::endl;
    //    std::cout << "Predicted Bias " << std::endl;
    //    printf("bias [%f, %f, %f] ", X_k(4), X_k(5), X_k(6));
    //}
    //std::cout << "S_k\n" << S_k << std::endl;
    //std::cout << "K_k\n" << K_k << std::endl;
    //std::cout << "P_k\n" << P_k << std::endl;
    //std::cout << "H\n" << H << std::endl;
    //
    //std::string tempstring;
    //std::cout << "Prediction done \n enter any key and press enter" << std::endl;
    //std::cin >> tempstring;
    previousT = T;
}

void KalmanFilter::Correction()
{
    Eigen::Matrix3d Triad_new;
    ComputeTriad(Mag_1, Acc_1, Triad_new);
    //std::cout << "9\n";
    //std::cout << "Newly computed Triad is \n" << Triad_new << std::endl;
    Eigen::Matrix3d RotationMatrix = Triad_new.transpose() * InitialTriad;
    //std::cout << "10\n";
    Eigen::Vector4d Quart;
    RotatationMatrix2Quarternion(RotationMatrix, Quart);
    //std::cout << "11\n";
    /*if(numberofPrints < max && numberofPrints > min)
        printf("corrected Quarternion is [%f,%f,%f,%f]\n", Quart(0), Quart(1), Quart(2), Quart(3)); */
    double Error_1 = (z_k - Quart).norm();
    double Error_2 = (z_k + Quart).norm();
    Eigen::Vector4d Error_in_prediction;
    //std::cout << "Error is" << Error_1 << std::endl;
   /* Eigen::Vector3d RPY;
    getRPY_1(X_k, RPY);*/
    //if (RPY(2) < 0.0 && RPY(2) > -180.00)
    //{
    //    std::cout << "These many cycles without any problem" << numberofPrints << std::endl;
    //    std::cout << "Mag input is [" << Mag_1(0) << "," << Mag_1(1) << "," << Mag_1(2) << "]" << std::endl;
    //    getRPY_1(z_k, RPY);
    //    std::cout << "predicted Yaw is " << RPY(2) << std::endl;
    //    getRPY_1(Quart, RPY);
    //    std::cout << "Corrected Yaw is " << RPY(2) << std::endl;
    //    std::cout << "predicted Quarternion is [" << z_k(0) << "," << z_k(1) << "," << z_k(2) << "," << z_k(3) << "]\n";
    //    std::cout << "Corrected Quarternion is [" << Quart(0) << "," << Quart(1) << "," << Quart(2) << "," << Quart(3) << "]\n";
    //    std::cout << "Error_1" << Error_1 << std::endl;
    //    std::cout << "Error_2" << Error_2 << std::endl;
    //    //std::string temp;
    //    //std::cin  >> temp;


    //}
    if (Error_1 > Error_2)
    {
        Error_in_prediction = z_k + Quart;
    }
        
    else
    {
        Error_in_prediction = Quart - z_k;
    }
        
        //std::cout << "12\n";
    X_k = X_k + K_k * (Error_in_prediction);
    X_k = X_k/ X_k.norm();
    Eigen::Matrix4d H;
    H << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    P_k = P_k - K_k * H * P_k;
    //std::cout << "13\n";
    numberofPrints++;
    data.quarternion_measurement[0] = X_k(0);
    data.quarternion_measurement[1] = X_k(1);
    data.quarternion_measurement[2] = X_k(2);
    data.quarternion_measurement[3] = X_k(3);
    WriteData2CSV();
  /*  std::cout << "Correction Step" << std::endl;
    std::cout << "X_k\n" << X_k << std::endl;
    std::cout << "Triad_new\n" << S_k << std::endl;
    std::cout << "P_k\n" << P_k << std::endl;
    std::cout << "Error_in_prediction\n" << Error_in_prediction << std::endl;
    std::cout << "Rotation Matrix \n" << RotationMatrix << std::endl;


    std::string tempstring;
    std::cout << "Correction done \n enter any key and press enter" << std::endl;
    std::cin >> tempstring;*/
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

    angles = angles * 180.0 / M_PI;
}
void KalmanFilter::getRPY(double* angles)
{
    Eigen::Vector4d q;
    q << X_k(0), X_k(1), X_k(2), X_k(3);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
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
    data.Time = Time;
}

void KalmanFilter::GetJacobian(Eigen::Vector3d U0, Eigen::Matrix4d& Jacobian)
{
    Jacobian = Eigen::Matrix4d::Zero();
    double w1, w2, w3;
    w1 = U0(0);
    w2 = U0(1);
    w3 = U0(2);

    Jacobian(0, 0) = 1;
    Jacobian(1, 0) = w1;
    Jacobian(2, 0) = w2;
    Jacobian(3, 0) = w3;

    Jacobian(0, 1) = -w1;
    Jacobian(1, 1) = 1;
    Jacobian(2, 1) = -w3;
    Jacobian(3, 1) = w2;

    Jacobian(0, 2) = -w2;
    Jacobian(1, 2) = w3;
    Jacobian(2, 2) = 1;
    Jacobian(3, 2) = -w1;

    Jacobian(0, 3) = -w3;
    Jacobian(1, 3) = -w2;
    Jacobian(2, 3) = w1;
    Jacobian(3, 3) = 1;
    
    Jacobian = Jacobian / 2;
}
void KalmanFilter::SetAngularVelocity(double w_x, double w_y, double w_z, long long Time)
{
    Gyro_w(0) = w_x;
    Gyro_w(1) = w_y;
    Gyro_w(2) = w_z;
    
    if (first_update)
    {
        WriteData2CSV();
        first_update = false;
    }

    data.gyr[0] = w_x;
    data.gyr[1] = w_y;
    data.gyr[2] = w_z;
    data.Time = Time;
    Prediction(Gyro_w, Time);
}

void KalmanFilter::SetMagnetometerMeasurements(double m_x, double m_y, double m_z)
{
    Mag_1(0) = m_x;
    Mag_1(1) = m_y;
    Mag_1(2) = m_z;
    data.mag[0] = m_x;
    data.mag[1] = m_y;
    data.mag[2] = m_z;

    
}

void KalmanFilter::SetAccelerometerMeasurements(double a_x, double a_y, double a_z)
{
    Acc_1(0) = a_x;
    Acc_1(1) = a_y;
    Acc_1(2) = a_z;
    //std::cout << "0-Acc\n";
    data.acc[0] = a_x;
    data.acc[1] = a_y;
    data.acc[2] = a_z;
    Correction();
    //std::cout << "Correction\n";
}


void KalmanFilter::RungeKuttaEval(Eigen::Vector4d& q0, long long T, Eigen::Vector3d U0) // also use previous T in Nano
{

    double dT = (T - previousT) * pow(10, -9);
    double w1, w2, w0;
    w0 = U0(0);
    w1 = U0(1);
    w2 = U0(2);

    Eigen::Matrix4d W;
    W <<  0.0, -w0, -w1, -w2,
            w0, 0.0, w2, -w1,
            w1, -w2, 0.0, w0,
            w2, w1, -w0, 0.0;
    W = W / 2;

    Eigen::Vector4d k1,k2,k3,k4;
    k1 = W * q0;
    k2 = W * (q0 + dT / 2 * k1);
    k3 = W * (q0 + dT / 2 * k2);
    k4 = W * (q0 + dT * k3);
    
    q0 = q0 + dT / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

    double Normq1 = q0.norm();
    q0 = q0 / Normq1;
}

void KalmanFilter::InitCSVFile()
{
    fout.open(file_name, std::ios::out | std::ios::app);
    fout << "Acc_x" << "," << "Acc_y" << "," << "Acc_z" << ","
         << "Mag_x" << "," << "Mag_y" << "," << "Mag_z" << ","
         << "Gyr_x" << "," << "Gyr_y" << "," << "Gyr_z" << ","
         << "Quarterion Predict_w" << "," << "Quarterion Predict_x" << "," << "Quarterion Predict_y" << "," << "Quarterion Predict_z" << ","
         << "Quarterion Correct_w" << "," << "Quarterion Correct_x" << "," << "Quarterion Correct_y" << "," << "Quarterion Correct_z" << ","
         << "quarternion_Gyro_w" << "," << "quarternion_Gyro_x" << "," << "quarternion_Gyro_y" << "," << "quarternion_Gyro_z" << ","
         << "TimeStamp" 
         << std::endl;
    fout.close();
}

void KalmanFilter::WriteData2CSV()
{
    fout.open(file_name, std::ios::out | std::ios::app);
    fout<< std::to_string(data.acc[0]) << "," << std::to_string(data.acc[1]) << "," << std::to_string(data.acc[2]) << ","
        << std::to_string(data.mag[0]) << "," << std::to_string(data.mag[1]) << "," << std::to_string(data.mag[2]) << ","
        << std::to_string(data.gyr[0]) << "," << std::to_string(data.gyr[1]) << "," << std::to_string(data.gyr[2]) << ","
        << std::to_string(data.quarternion_predict[0])     << "," << std::to_string(data.quarternion_predict[1]) << ","  << std::to_string(data.quarternion_predict[2]) << "," << std::to_string(data.quarternion_predict[3]) << ","
        << std::to_string(data.quarternion_measurement[0]) << "," << std::to_string(data.quarternion_measurement[1])  << "," << std::to_string(data.quarternion_measurement[2]) << "," << std::to_string(data.quarternion_measurement[3]) << ","
        << std::to_string(data.quarternion_Gyro_pure[0]) << "," << std::to_string(data.quarternion_Gyro_pure[1]) << "," << std::to_string(data.quarternion_Gyro_pure[2]) << "," << std::to_string(data.quarternion_Gyro_pure[3]) << ","
        << std::to_string(data.Time)
        << std::endl;
    data = CSVData();
    fout.close();
}