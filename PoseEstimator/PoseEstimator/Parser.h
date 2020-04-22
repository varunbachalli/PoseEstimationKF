#include "Server.h"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <fstream>
#include "KalmanFilter.h"
#include "Magnetometer_Calibration.h"
#include "InitialValues.h"
#include "KalmanFilter.h"
#include <map>
class Parser
{
public: 
	Parser();
	void run();
	void setMutex(std::mutex* m);
	void clearLatestMeasurements();
	void setValue(std::string serverText);
	void setServer(Server* server);
	void WriteCalibrationMeasurement(std::array<double , 3> measurement);
	void WriteKalmanFilterMeasurement(std::array<double, 3> measurement, long long Time, char Type);
	
	void ExecuteKalmanFilter();
private :

	std::condition_variable cond;
	std::condition_variable* cv;
	void NormalizeValues(double* x, double* y, double* z);
	void WriteCSV(std::array<double, 3> measurement, long long time, char sensorType);
	char phase;

	std::array<double, 3> gyro;
	std::array<double, 3> acc_0;
	std::array<double, 3> mag_0;
	std::array<double, 3> acc_1;
	std::array<double, 3> mag_1;
	
	long long time_acc_0;
	long long time_mag_0;
	long long time_acc_1;
	long long time_mag_1;
	long long time_gyro;


	bool mag_1_is_set = false;
	bool acc_1_is_set = false;
	bool gyro_is_set = false;

	std::vector<std::string> serverInput;
	std::mutex* m1;
	std::mutex ServerMutex;
	void ProcessString(std::string& c);

	void initialMeanAndCovariance(std::array<double, 3> measurement, char sensorType);
	
	//void KalmanFilter();
	std::array<double, 3> LinearInterpolationSensor(long long t1, long long t2, long long t3, std::array<double, 3> y1, std::array<double, 3> y2);
	void setAcc0(std::array<double, 3> measruement, long long Time);
	void setAcc1(std::array<double, 3> measruement, long long Time);
	void setMag0(std::array<double, 3> measruement, long long Time);
	void setMag1(std::array<double, 3> measruement, long long Time);
	void setGyr(std::array<double, 3> measruement, long long Time);
	std::string FindValues(std::string& str, std::string regex);

	LatestMeasurements latestMeasurment;
	Server* s;


	//const std::string file_source = "D:\\GITProjects\\Kalman Filtering Server\\PoseEstimationKF\\Sensor_CSV\\SensorReading.csv";
	const std::string file_source = "SensorReading.csv";
	std::fstream fout;
	std::once_flag flag_;
	std::vector<std::future<void>> CSV_Futures;
	std::mutex csvMutex;


	bool FirstAccSet = false;
	bool FirstMagSet = false;

	Magnetometer_Calibration Mag_calib;
	InitialValues init_acc;
	InitialValues init_mag;
	InitialValues init_gyro;
	KalmanFilter kalmanFilter;

	std::array<double, 3> avg_gyr, avg_acc, avg_mag, variance_gyr, variance_acc, variance_mag;

	int valuesInitDone = 0;
	long long latestSensorTime = 0;
};

