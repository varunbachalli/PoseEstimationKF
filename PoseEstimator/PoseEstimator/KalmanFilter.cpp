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

   /* std::cout << "initial Values are" << std::endl;
    std::cout << "mag_0 \n" << Mag_0 << std::endl;
    std::cout << "Acc_0 \n" << Acc_0 << std::endl;
    std::cout << "R_Sigma \n" << std::endl;
    for (double k : R_Sigma)
    {
        std::cout << k << ',';
    }
    std::cout << "\n";
    std::cout << "Gyro_drift_0 \n" << Gyro_drift_0 << std::endl;
    std::cout << "Gyro_Sigma \n" << Gyro_Sigma << std::endl;*/

    

	X_k << 1.0, 0.0, 0.0, 0.0, Gyro_drift_0(0), Gyro_drift_0(1), Gyro_drift_0(2);
    
	Q_k = Eigen::MatrixXd::Identity(7, 7);
    double temp[] = { 0.01, 0.01,  0.01,  0.01, Gyro_Sigma(0), Gyro_Sigma(1), Gyro_Sigma(2) };
    for (int i = 0; i < 7; ++i)
    {
        Q_k(i, i) = temp[i];
    }

    double MaxElement =  *std::max_element(R_Sigma.begin(), R_Sigma.end());
   
    R_k = Eigen::Matrix4d::Identity(4, 4);
    R_k = R_k * MaxElement;
    InitialTriad = ComputeTriad(Mag_0, Acc_0);
    /*std::cout << "InitialTriad\n" << InitialTriad << std::endl;*/
    P_k = Eigen::MatrixXd::Identity(7, 7);

    /*std::cout << "InitialTriad\n" << InitialTriad << std::endl;
    std::cout << "P_k\n" << P_k << std::endl;
    std::cout << "R_k\n" << R_k << std::endl;
    std::cout << "Q_k\n" << Q_k << std::endl;
    std::cout << "X_k\n" << X_k << std::endl;
    std::string tempstring;
    std::cout << "Initialization done \n enter any key and press enter" << std::endl;
    std::cin >> tempstring;*/
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
    Eigen::Matrix<double, 7, 7> Jacobian = GetJacobian(X_k, Gyro, T);
    P_k = Jacobian * P_k * Jacobian.transpose() + Q_k;
    X_k = RungeKuttaEval(X_k, T, Gyro);
    Eigen::Vector4d QuartTemp;
    QuartTemp << X_k(0), X_k(1), X_k(2), X_k(3);
   

    Eigen::Matrix<double, 4, 7> H;
    H << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0;
    S_k = H * P_k * H.transpose() + R_k;
    K_k = P_k * H.transpose() * S_k.inverse();
   
    z_k = H * X_k;
    //std::cout << "Prediction Step"<< std::endl;
    if (numberofPrints < max && numberofPrints > min)
    {
        /*std::cout << "\n K_k is \n" << K_k << std::endl;*/
        printf("predicted Quarternion is [%f,%f,%f,%f]\n", z_k(0), z_k(1), z_k(2), z_k(3));
        std::cout << "Predicted Yaw" << getRPY_1(QuartTemp)(2) << std::endl;
        std::cout << "Predicted Bias " << std::endl;
        printf("bias [%f, %f, %f] ", X_k(4), X_k(5), X_k(6));
    }
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
    if(numberofPrints < max && numberofPrints > min)
        printf("corrected Quarternion is [%f,%f,%f,%f]\n", Quart(0), Quart(1), Quart(2), Quart(3)); 
    double Error_1 = (z_k - Quart).norm();
    double Error_2 = (z_k + Quart).norm();
    Eigen::Vector4d Error_in_prediction;
    //std::cout << "Error is " << Error_1 << std::endl;
    if (Error_1 > Error_2)
    {
        Error_in_prediction = z_k + Quart;
        std::cout << "Error_1 is being used" << std::endl;
    }
        
    else
        Error_in_prediction = z_k - Quart;

    X_k = X_k + K_k * (Error_in_prediction);
    P_k = P_k - K_k * S_k * K_k.transpose();
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

    //std::cout << "input yaw rate is " << w3 << std::endl;

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
    q_k1 = q_k1 / q_k1.norm();
    // yn+1 = yn + 1/6 * h * (k1 + 2k2 + 2k3 + k4)
    X_0(0 ,0) = q_k1(0, 0);
    X_0(1, 0) = q_k1(1, 0);
    X_0(2, 0) = q_k1(2, 0);
    X_0(3, 0) = q_k1(3, 0);

    //std::cout <<"dt is = " << T_ << std::endl;
    return X_0;

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

int main()
{
    std::ifstream file;
    file.open("D:/GITProjects/Kalman Filtering Server/PoseEstimationKF/Sensor_CSV/MagnetometerReadings.csv");
    if (!file)
    {
        std::cerr << "couldn't open file" << std::endl;
    }
    
    Reading acc;
    acc.x = 0.0;
    acc.y = 0.0;
    acc.z = -1.0;

    Reading Gyro;
    Gyro.x = 0.0;
    Gyro.y = 0.0;
    Gyro.z = M_PI/2;
    KalmanFilter k; 

    for (int i = 0; i < 11999; ++i)
    {
        
        Reading m = MagCSVFile(&file);
        

        if (i == 0)
        {
            /*
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
            */
            std::map<std::string, std::array<double, 3>> values;
            values["mag0"] = {m.x,m.y,m.z};
            values["acc0"] = {acc.x,acc.y,acc.z};
            values["mag_sig"] = {1.0,0.01,0.01};
            values["acc_sig"] = {1.0,0.01,0.01};
            values["gyro_drift_0"] = {0.0,0.0,0.0};
            values["gyro_drift_sig"] = {0.1,0.1,0.1};
            k = KalmanFilter(values,m.T);
        }
        else
        {
            Gyro.T = m.T;
            k.SetAngularVelocity(Gyro.x, Gyro.y, Gyro.z, Gyro.T);
            k.SetMagnetometerMeasurements(m.x, m.y, m.z);
            //printf("Magvalues are [%f, %f, %f]", m.x, m.y, m.z);
            k.SetAccelerometerMeasurements(acc.x, acc.y, acc.z);
            Eigen::Vector3d rpy = k.getRPY();
           // printf("yaw is %f \n", rpy(2)*180/M_PI);
            if (rpy(2) > M_PI / 180 * 15.0)
            {
                std::string tempstring;
                std::cout << "Prediction done \n enter any key and press enter" << std::endl;
                std::cin >> tempstring;
            }
        }

    }

    file.close();

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