#include "Parser.h"


// #### 
// IMPORTANT !!! 
// ####

// Magnetometer is left handed 
// solution : multiply y reading by -1




void Parser::ProcessString(std::string& str)
{
	phase =FindValues(str, ",")[0];
	SensorReading sensor;
	sensor.type = FindValues(str, ":")[0];
	sensor.x = std::stod(FindValues(str, ","));
	sensor.y = std::stod(FindValues(str, ","));
	sensor.z = std::stod(FindValues(str, ","));
	sensor.time_stamp = std::stol(FindValues(str, "t:"));
	std::thread csvThread(&Parser::WriteCSV, this, sensor);
	// Kalman Filter, Calibration 
	// join
	
	// phase 1, 2, 3

	switch (phase)
	{
	case '1':
		// calibrate mag
		break;
	case '2':
		// calibarte all three
		break;
	case '3':
		// run Kalman Filter
		break;
	default:
		break;
	}
	csvThread.join();
}


void Parser::WriteCalibrationMeasurement(SensorReading measurement)
{
	
}
void Parser::WriteKalmanFilterMeasurement(SensorReading measurement)
{
	/*
	0 - Acc, 1 - gyr, 2- mag
	*/

	if ((acc_0.time_stamp == 0) || (mag_0.time_stamp == 0)) // init
	{
		if (measurement.type == '0')
		{
			setAcc0(measurement);
			std::cout << "init acc0" << std::endl;
		}
		else if (measurement.type == '2')
		{
			setMag0(measurement);
			std::cout << "init mag0" << std::endl;
		}
	}
	else // after
	{
		if (gyro_is_set == false)
		{
			if (measurement.type == '0')
			{
				setAcc0(measurement);
				std::cout << "after acc0" << std::endl;
			}

			else if (measurement.type == '2')
			{
				setMag0(measurement);
				std::cout << "after mag0" << std::endl;
			}
			else
			{
				setGyr(measurement);
				gyro_is_set = true;
				std::cout << "gyro set" << std::endl;
			}
		}

		else 
		{
			if (measurement.type == '0')
			{
				setAcc1(measurement);
				acc_1_is_set = true;
				std::cout << "gyro set acc1" << std::endl;
			}

			else if (measurement.type == '2')
			{
				setMag1(measurement);
				mag_1_is_set = true;
				std::cout << "gyro set mag1" << std::endl;
			}
			else
			{
				setGyr(measurement);
				if (acc_1_is_set)
				{
					setAcc0(acc_1);
					std::cout << "gyro reset acc1 set acc0" << std::endl;
				}

				if (mag_1_is_set)
				{
					setMag0(mag_1);
					std::cout << "gyro reset mag_1 set mag0" << std::endl;

				}
				acc_1_is_set = false;
				mag_1_is_set = false;
				

			}
		}

		if (acc_1_is_set == true && mag_1_is_set == true)
		{
			gyro_is_set = false;
			acc_1_is_set = false;
			mag_1_is_set = false;

			// set data and run kalman filter
			KalmanFilter();

			setAcc0(acc_1);
			setMag0(mag_1);
			setGyr(SensorReading());
		}
	}
}

void Parser::KalmanFilter()
{
	
	///*
	//Test : 
	//*/

	std::vector<SensorReading> sensorReadings_test;
	for (int i = 1; i <= 5; ++i)
	{
		SensorReading test;
		test.x = i * 10.0;
		test.y = i * 20.0;
		test.z = i * 30.0;
		test.time_stamp = i * 10000;
		sensorReadings_test.push_back(test);
	}

	gyro = sensorReadings_test[0];
	acc_0 = sensorReadings_test[1];
	mag_0 = sensorReadings_test[2];   
	acc_1 = sensorReadings_test[3];   
	mag_1 = sensorReadings_test[4];   


	latestMeasurment.time_stamp = gyro.time_stamp;
	latestMeasurment.gyro[0] = gyro.x;
	latestMeasurment.gyro[1] = gyro.y;
	latestMeasurment.gyro[2] = gyro.z;

	double* acc = &latestMeasurment.acc[0];
	double* mag = &latestMeasurment.mag[0];
	
	//std::cout << "acc" << std::endl;
	*acc = LinearInterpolationSensor(acc_0.time_stamp, acc_1.time_stamp, acc_0.x, acc_1.x, latestMeasurment.time_stamp);
	//std::cout << "mag" << std::endl;
	*mag = LinearInterpolationSensor(mag_0.time_stamp, mag_1.time_stamp, mag_0.x, mag_1.x, latestMeasurment.time_stamp);
	++acc;
	++mag;
	
	//std::cout << "acc" << std::endl;
	*acc = LinearInterpolationSensor(acc_0.time_stamp, acc_1.time_stamp, acc_0.y, acc_1.y, latestMeasurment.time_stamp);
	//std::cout << "mag" << std::endl;
	*mag = LinearInterpolationSensor(mag_0.time_stamp, mag_1.time_stamp, mag_0.y, mag_1.y, latestMeasurment.time_stamp);
	++acc;
	++mag;

	//std::cout << "acc" << std::endl;
	*acc = LinearInterpolationSensor(acc_0.time_stamp, acc_1.time_stamp, acc_0.z, acc_1.z, latestMeasurment.time_stamp);
	//std::cout << "mag" << std::endl;
	*mag = LinearInterpolationSensor(mag_0.time_stamp, mag_1.time_stamp, mag_0.z, mag_1.z, latestMeasurment.time_stamp);

	//std::cout << "accelerometer :";
	// TODO:: 
	// 1. compute Kalman Filter
	// 2. plot on opengl
	/*for (double d : latestMeasurment.acc)
	{
		std::cout << d << ",";
	}

	std::cout << std::endl;
	

	std::cout << "magnetometer :";
	for (double d : latestMeasurment.mag)
	{
		std::cout << d << ",";
	}*/

	
	/*printf("Kalman filter called [acc_0, mag_0, gyr, acc_1, mag_1] = [%d, %d, %d, %d, %d]", acc_0.time_stamp, mag_0.time_stamp, gyro.time_stamp, acc_1.time_stamp, mag_1.time_stamp);*/
	
}

double Parser::LinearInterpolationSensor(long& t1, long& t2, double& y1, double& y2, long& t3)
{
	/*printf("[t1,t2,y1,t2,t3] = [%d, %d,%f, %f, %d]", t1, t2, y1, y2, t3);
	std::cout << std::endl;
	printf("((y2 - y1) / ((double)t2 - (double)t1)) * ((double)t3) + y1; %f", ((y2 - y1) / ((double)t2 - (double)t1)) * ((double)t3) + y1);
	std::cout << std::endl;*/
	return ((y2 - y1) / ((double)t2 - (double)t1)) * ((double)t3) + y1;
}
void Parser::setAcc0(SensorReading sensor)
{
	acc_0.x = sensor.x;
	acc_0.y = sensor.y;
	acc_0.z = sensor.z;
	acc_0.time_stamp = sensor.time_stamp;
}

void Parser::setAcc1(SensorReading measurement)
{
	acc_1.x = measurement.x;
	acc_1.y = measurement.y;
	acc_1.z = measurement.z;
	acc_1.time_stamp = measurement.time_stamp;
}
void Parser::setMag0(SensorReading measurement)
{
	mag_0.x = measurement.x;
	mag_0.y = measurement.y;
	mag_0.z = measurement.z;
	mag_0.time_stamp = measurement.time_stamp;
}
void Parser::setMag1(SensorReading measurement)
{
	mag_1.x = measurement.x;
	mag_1.y = measurement.y;
	mag_1.z = measurement.z;
	mag_1.time_stamp = measurement.time_stamp;
}
void Parser::setGyr(SensorReading measurement)
{
	gyro.x = measurement.x;
	gyro.y = measurement.y;
	gyro.z = measurement.z;
	gyro.time_stamp = measurement.time_stamp;
}


std::string Parser::FindValues(std::string& str , std::string regex)
{
	std::smatch m;
	std::regex_search(str, m, std::regex(regex));
	std::string returnString;
	if (regex == "t:")
	{
		returnString = m.suffix().str();
		str = "";
	}
	else
	{
		returnString = m.prefix().str();
		str = m.suffix().str();
	}
	
	return returnString;
}

void Parser::WriteCSV(SensorReading Measurement)
{
	std::call_once(flag_, [&]() {
		fout.open(file_source, std::ios::out | std::ios::app);
		fout << "Sensor" <<"," <<"x"<< "," << "y" << "," << "z" << "," << "time_stamp" <<std::endl; 
								});
	fout << Measurement.type << "," << Measurement.x << "," << Measurement.y << "," << Measurement.z << "," << Measurement.time_stamp << std::endl;
}

void Parser::run()
{
	cv = s->getCV(); // pointer to condition variable
	while (true)
	{
		std::unique_lock<std::mutex> locker(*m1); // access mutex by reference
		cv->wait(locker, [&]() {return !s->getSensorReadings().empty();});
		std::vector<std::string> readings = s->getSensorReadings();
		s->clearSensorReadings();
		locker.unlock();
		std::cout << "evaluating :\t";
		for (std::string Measurement : readings)
		{
			ProcessString(Measurement);


			/*
			check stages in the first string element. i.e.
			1. Mag calibration
			2. Acc and Gyro calibration
			3. Kalman Filtering
			*/
		}

		if (s->getSensorReadings().empty())
		{
			break;
		}
	}
	std::cout << "out of loop" << std::endl;
	
}

void Parser::setMutex(std::mutex* m)
{
	m1 = m;
}

void Parser::clearLatestMeasurements()
{
	latestMeasurment = {};
}

void Parser::setValue(std::string serverText)
{
	std::unique_lock<std::mutex> locker(ServerMutex);
	serverInput.push_back(serverText);
	cond.notify_one();
}

void Parser::setServer(Server* server)
{
	s = server;
}




