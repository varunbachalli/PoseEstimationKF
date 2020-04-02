#include "Server.h"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <fstream>
#include "KalmanFilter.h"

#include "CudaAbstraction.cuh"




class Parser
{
public: 
	Parser();
	void run();
	void setMutex(std::mutex* m);
	void clearLatestMeasurements();
	void setValue(std::string serverText);
	void setServer(Server* server);
	void WriteCalibrationMeasurement(SensorReading measurement);
	void WriteKalmanFilterMeasurement(SensorReading measurement);// change back to private
	void KalmanFilter();
private :
	std::condition_variable cond;
	std::condition_variable* cv;
	void WriteCSV(SensorReading Measurement);
	char phase;
	/*struct SensorReading
	{
		char type;
		double x;
		double y;
		double z;
		long time_stamp;
	};*/

	SensorReading gyro;
	SensorReading acc_0;
	SensorReading mag_0;
	SensorReading acc_1;
	SensorReading mag_1;
	bool mag_1_is_set = false;
	bool acc_1_is_set = false;
	bool gyro_is_set = false;

	std::vector<std::string> serverInput;
	std::mutex* m1;
	std::mutex ServerMutex;
	void ProcessString(std::string& c);
	
	//void KalmanFilter();
	double LinearInterpolationSensor(long& t1, long& t2, double& y1, double& y2, long& t3);
	void setAcc0(SensorReading sensor);
	void setAcc1(SensorReading sensor);
	void setMag0(SensorReading sensor);
	void setMag1(SensorReading sensor);
	void setGyr(SensorReading sensor);
	std::string FindValues(std::string& str, std::string regex);

	LatestMeasurements latestMeasurment;
	Server* s;


	//const std::string file_source = "D:\\GITProjects\\Kalman Filtering Server\\PoseEstimationKF\\Sensor_CSV\\SensorReading.csv";
	const std::string file_source = "SensorReading.csv";
	std::fstream fout;
	std::once_flag flag_;
	std::vector<std::future<void>> CSV_Futures;
	std::mutex csvMutex;
};

