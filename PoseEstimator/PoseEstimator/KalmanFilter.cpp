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
KalmanFilter::KalmanFilter(){}

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
    X_k << 1.0, 0.0, 0.0, 0.0;
    
	Q_k = Eigen::MatrixXd::Identity(4, 4);
    double temp[] = { 0.01, 0.01,  0.01,  0.01 };  
    for (int i = 0; i < 4; ++i)
    {
        Q_k(i, i) = temp[i];
    }
    double MaxElement =  *std::max_element(R_Sigma.begin(), R_Sigma.end());
   
    R_k = Eigen::Matrix4d::Identity(4, 4);
    R_k = R_k * MaxElement;
    InitialTriad = ComputeTriad(Mag_0, Acc_0);
    P_k = Eigen::MatrixXd::Identity(4, 4);
    P_k = 0.5 * P_k;
}

Eigen::Matrix3d KalmanFilter::ComputeTriad(Eigen::Vector3d s, Eigen::Vector3d m)
{
    Eigen::Matrix3d Triad;
    Eigen::Vector3d t1 = s/s.norm();
    Eigen::Vector3d t2 = s.cross(m);
    t2 = t2 / t2.norm();
    Eigen::Vector3d t3 = t2.cross(t1);
    Triad << t1(0), t2(0), t3(0),
             t1(1), t2(1), t3(1),
             t1(2), t2(2), t3(2);
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
    Eigen::Matrix<double, 4, 4> Jacobian = GetJacobian(Gyro);
    P_k = Jacobian * P_k * Jacobian.transpose() + Q_k;
    Eigen::Vector4d PrevX_k = X_k;
    X_k = RungeKuttaEval(X_k, T, Gyro);
    Eigen::Vector3d Change_in_angle = getRPY_1(X_k - PrevX_k);
    /*printf("Change_in_angle is [%f,%f,%f]\n", Change_in_angle(0), Change_in_angle(1), Change_in_angle(2));*/
    /*std::cout << "change in angle is " << Change_in_angle(2) << std::endl;*/

    Eigen::Matrix4d H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    S_k = H * P_k * H.transpose() + R_k;
    K_k = P_k * H.transpose() * S_k.inverse();
    z_k = H * X_k;

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
    Triad_new = ComputeTriad(Mag_1, Acc_1);
    //std::cout << "Newly computed Triad is \n" << Triad_new << std::endl;
    Eigen::Matrix3d RotationMatrix = Triad_new.transpose() * InitialTriad;
    Eigen::Vector4d Quart = RotatationMatrix2Quarternion(RotationMatrix);
    /*if(numberofPrints < max && numberofPrints > min)
        printf("corrected Quarternion is [%f,%f,%f,%f]\n", Quart(0), Quart(1), Quart(2), Quart(3)); */
    double Error_1 = (z_k - Quart).norm();
    double Error_2 = (z_k + Quart).norm();
    Eigen::Vector4d Error_in_prediction;
    //std::cout << "Error is" << Error_1 << std::endl;

    if (getRPY_1(X_k)(2) < 0.0 && getRPY_1(X_k)(2) > -180.00)
    {
        std::cout << "These many cycles without any problem" << numberofPrints << std::endl;
        std::cout << "predicted Yaw is " << getRPY_1(z_k) << std::endl;
        std::cout << "Corrected Yaw is " << getRPY_1(Quart) << std::endl;
        printf("predicted Quarternion is [%f,%f,%f,%f]\n", z_k(0), z_k(1), z_k(2), z_k(3));
        printf("Corrected Quarternion is [%f,%f,%f,%f]\n", Quart(0), Quart(1), Quart(2), Quart(3));
        std::cout << "Error_1" << Error_1 << std::endl;
        std::cout << "Error_2" << Error_2 << std::endl;
        //std::string temp;
        //std::cin  >> temp;


    }
    if (Error_1 > Error_2)
    {
        Error_in_prediction = z_k + Quart;
    }
        
    else
        Error_in_prediction = Quart - z_k;

    X_k = X_k + K_k * (Error_in_prediction);
    Eigen::Matrix4d H;
    H << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    P_k = P_k - K_k * H * P_k;
    numberofPrints++;
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

Eigen::Vector4d KalmanFilter::getXk_1()
{
    Eigen::Vector4d quart;
    quart << X_k(0), X_k(1), X_k(2), X_k(3);
    return quart;
}

Eigen::Vector3d KalmanFilter::getRPY_1(Eigen::Vector4d q)
{
    Eigen::Vector3d angles;
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
    return angles;
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




Eigen::Matrix4d KalmanFilter::GetJacobian(Eigen::Vector3d U0)
{
    Eigen::Matrix4d Jacobian;
    double w1, w2, w3;
    w1 = U0(0);
    w2 = U0(1);
    w3 = U0(2);
    Jacobian << 1,      -w1,   -w2,   -w3,
                w1,      1,     w3,   -w2,
                w2,     -w3,    1,     w1,
                w3,      w2,   -w1,    1;        
    Jacobian = Jacobian / 2;
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


Eigen::Vector4d KalmanFilter::RungeKuttaEval(Eigen::Vector4d q0, double T, Eigen::Vector3d U0) // also use previous T in Nano
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
    Eigen::Vector4d q1;
    q1 = q0 + dT / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

    double Normq1 = q1.norm();
    q1 = q1 / Normq1;
    return q1;

}

struct Reading
{
    double x;
    double y;
    double z;
    long T;
};

Reading MagCSVFile(std::ifstream* file)
{
    std::string outstring;
    *file >> outstring;
    std::stringstream s_stream(outstring);
    std::string x[4];
    for (int j = 0; j < 4; ++j)
    {
        std::string substr;
        getline(s_stream, substr, ',');
        x[j] =  substr;
    }
    Reading reading;
    reading.x = std::stod(x[0]);
    reading.y = std::stod(x[1]);
    reading.z = std::stod(x[2]);
    reading.T = std::stol(x[3]);
    return reading;

}

void getEigenVector(Eigen::Matrix3d& mat) 
{
    mat = Eigen::Matrix3d::Identity();
}

int main()
{
    //std::ifstream file;
    //file.open("D:/GITProjects/Kalman Filtering Server/PoseEstimationKF/Sensor_CSV/MagnetometerReadings.csv");
    //if (!file)
    //{
    //    std::cerr << "couldn't open file" << std::endl;
    //}
    //std::ofstream writefile;
    //writefile.open("KalmanFilter_results.txt");
    //if (!writefile)
    //{
    //    std::cerr << "couldn't open writefile file" << std::endl;
    //}
    //Reading acc;
    //acc.x = 0.0;
    //acc.y = 0.0;
    //acc.z = -1.0;

    //Reading Gyro;
    //Gyro.x = 0.0;
    //Gyro.y = 0.0;
    //Gyro.z = M_PI/2;
    //KalmanFilter k; 
    //Eigen::Vector3d rpy; std::string output;
    //for (int i = 0; i < 11999; ++i)
    //{
    //    
    //    Reading m = MagCSVFile(&file);
    //    

    //    if (i == 0)
    //    {
    //        std::map<std::string, std::array<double, 3>> values;
    //        values["mag0"] = {m.x,m.y,m.z};
    //        values["acc0"] = {acc.x,acc.y,acc.z};
    //        values["mag_sig"] = {0.10,0.01,0.01};
    //        values["acc_sig"] = {0.10,0.01,0.01};
    //        values["gyro_drift_0"] = {0.0,0.0,0.0};
    //        values["gyro_drift_sig"] = {0.1,0.1,0.1};
    //        k = KalmanFilter(values,m.T);
    //    }
    //    else
    //    {
    //        Gyro.T = m.T;
    //        k.SetAngularVelocity(Gyro.x, Gyro.y, Gyro.z, Gyro.T);
    //        k.SetMagnetometerMeasurements(m.x, m.y, m.z);
    //        //printf("Magvalues are [%f, %f, %f]", m.x, m.y, m.z);
    //        k.SetAccelerometerMeasurements(acc.x, acc.y, acc.z);
    //        rpy = k.getRPY();
    //        //printf("yaw is %f \n", rpy(2)*180/M_PI);
    //        double yaw = rpy(2) * 180 / M_PI;
    //        output = std::to_string(yaw);
    //        output = output + "\n";
    //        writefile << output;
    //        if (i == 1974) std::cout << "1\n";
    //        
    //        //std::cout << "yaw" << output;
    //        /*if (rpy(2) > M_PI / 180 * 90.0)
    //        {
    //            std::string tempstring;
    //            std::cout << "Prediction done \n enter any key and press enter" << std::endl;
    //            std::cin >> tempstring;
    //        }*/
    //        if (i % 1000 == 0)
    //        {
    //            std::cout << i << std::endl;
    //        }
    //    }

    //}

    //file.close();
    //writefile.close();

    Eigen::Matrix3d a;
    std::cout << "pre" << a << std::endl;
    getEigenVector(a);
    std::cout << "post" << a << std::endl;

    _getch();
    return 0;
}

/*
std::string tempstring;
    std::cout << "Prediction done \n enter any key and press enter" << std::endl;
    std::cin >> tempstring;

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