#include "Parser.h"



/*

TODO: 
1. Measure the error in the quarternions between predicted value and measurement value in kalman filter.
2. store readings into CSV file
3. Store Initial Values
4. Store Kalman Filter input

*/
Parser::Parser()
{
	init_acc = InitialValues(300);
	init_mag = InitialValues(300);
	init_gyro = InitialValues(300);
	Mag_calib = Magnetometer_Calibration();
	std::cout << "initial values initialized " << std::endl;
}

void Parser::ProcessString(std::string str)
{
	if (str.size() > 30)
	{

		phase = str[0];
		str = str.substr(2, str.size() - 2);
		std::array<double, 3> sensor_reading;
		long long Time;
		char Type;
		Type = FindValues(str, ":")[0];
		sensor_reading[0] = std::stod(FindValues(str, ","));
		sensor_reading[1] = std::stod(FindValues(str, ","));
		sensor_reading[2] = std::stod(FindValues(str, ","));
		Time = std::stoll(FindValues(str, "t:"));

		//std::thread csvThread(&Parser::WriteCSV, this, sensor_reading, Time, Type);


		switch (phase)
		{
		case '1':
			if (!Mag_calib.MagnetometerCalibrated())
			{
				/*std::cout << "calibrating" << std::endl;
				printf("  time = %lld, sensor readings = [%f,%f,%f] \n", Time, sensor_reading[0], sensor_reading[1], sensor_reading[2]);*/
				WriteCalibrationMeasurement(sensor_reading);
			}
			else
			{
				//printf("before sensor readings = [%f,%f,%f] \n", sensor_reading[0], sensor_reading[1], sensor_reading[2]);
				Mag_calib.CorrectValues(sensor_reading[0], sensor_reading[1], sensor_reading[2]);
				//printf("after sensor readings = [%f,%f,%f] \n", sensor_reading[0], sensor_reading[1], sensor_reading[2]);
			}

			break;
		case '2':

			if (!Acc_initialized || !Mag_initialized || !Gyr_initialized)
			{
				initialMeanAndCovariance(sensor_reading, Type);
			}
			else if (Acc_initialized && Mag_initialized && Gyr_initialized && !Kalman_initialized)
			{
				std::cout << "Initial values in Values set \n";
				std::map<std::string, std::array<double, 3>> Values;
				std::array<double, 3> temp_mag;
				std::array<double, 3> temp_acc;
				memcpy(&temp_mag, &avg_mag, sizeof(double) * 3);
				memcpy(&temp_acc, &avg_acc, sizeof(double) * 3);
				NormalizeValues(temp_mag[0], temp_mag[1], temp_mag[2]);
				NormalizeValues(temp_acc[0], temp_acc[1], temp_acc[2]);
				Values["mag0"] = temp_mag;
				Values["acc0"] = temp_acc;
				Values["mag_sig"] = variance_mag;
				Values["acc_sig"] = variance_acc;
				Values["gyro_drift_0"] = avg_gyr;
				Values["gyro_drift_sig"] = variance_gyr;
				kalmanFilter = KalmanFilter(Values, Time);
				
				setAcc0(avg_acc, Time);
				setMag0(avg_mag, Time);
				Kalman_initialized = true;

				//printf("average acc [%f,%f,%f]\n", avg_acc[0], avg_acc[1], avg_acc[2]);
				//printf("average mag [%f,%f,%f]\n", avg_mag[0], avg_mag[1], avg_mag[2]);
			}

			else
			{
				/*std::cout << "Values updating \n";
				printf("average acc [%f,%f,%f]\n", avg_acc[0], avg_acc[1], avg_acc[2]);
				printf("average mag [%f,%f,%f]\n", avg_mag[0], avg_mag[1], avg_mag[2]);*/
				
				setAcc0(avg_acc, Time);
				setMag0(avg_mag, Time);
				std::cout << Time << std::endl;
				kalmanFilter.UpdateLatestPreviousTime(Time);
			}
			break;

		case '3':
			WriteKalmanFilterMeasurement(sensor_reading, Time, Type);
			break;
		default:
			break;
		}
	}
	else
	{
		std::cout << "some weird shit going on" << std::endl;
		std::cout << previous_string << std::endl;
		std::cout << str << std::endl;
		
	}
	previous_string = str;
	//csvThread.join();
}

void Parser::initialMeanAndCovariance(std::array<double, 3> measurement, char type)
{
	if (type == '0')
	{
		if (!init_acc.sensorCalibrated())
		{
			init_acc.setValuesforAverage(measurement[0], measurement[1], measurement[2]);
			
			acc_count++;
		}
		else
		{
			if (!Acc_initialized)
			{
				double avg[3];
				double variance[3];
				init_acc.getAverageValues(avg);
				init_acc.getVariance(variance);
				std::copy(std::begin(avg), std::end(avg), avg_acc.begin());
				std::copy(std::begin(variance), std::end(variance), variance_acc.begin());
				std::cout << "acc average set" << std::endl;
				printf("average is [%f, %f, %f]", avg[0], avg[1], avg[2]);
				printf("variance is [%f, %f, %f]", variance[0], variance[1], variance[2]);

				Acc_initialized = true;
				std::string text = "Average Acc = [" + std::to_string(avg[0]) + "," + std::to_string(avg[1]) + "," + std::to_string(avg[2]) + "]\n";
				WriteTextFile(text);
				text = "Variance Acc = [" + std::to_string(variance[0]) + "," + std::to_string(variance[1]) + "," + std::to_string(variance[2]) + "]\n";
				WriteTextFile(text);
			}
		}
	}	// set values std::copy(std::begin(arr), std::end(arr), first.begin());
	else if (type == '2')
	{
		if (!init_mag.sensorCalibrated())
		{
			Mag_calib.CorrectValues(measurement[0], measurement[1], measurement[2]);
			init_mag.setValuesforAverage(measurement[0], measurement[1], measurement[2]);
			
			mag_count++;
		}
		else
		{
			if (!Mag_initialized)
			{
				double avg[3];
				double variance[3];
				init_mag.getAverageValues(avg);
				init_mag.getVariance(variance);
				std::copy(std::begin(avg), std::end(avg), avg_mag.begin());
				std::copy(std::begin(variance), std::end(variance), variance_mag.begin());
				Mag_initialized = true;

				std::string text = "Average Mag = [" + std::to_string(avg[0]) + "," + std::to_string(avg[1]) + "," + std::to_string(avg[2]) + "]\n";
				WriteTextFile(text);
				text = "Variance Mag = [" + std::to_string(variance[0]) + "," + std::to_string(variance[1]) + "," + std::to_string(variance[2]) + "]\n";
				WriteTextFile(text);
				std::cout << "mag average set" << std::endl;
				printf("average is [%f, %f, %f]", avg[0], avg[1], avg[2]);
				printf("variance is [%f, %f, %f]", variance[0], variance[1], variance[2]);
			}
		}

	}
	else if (type == '1')
	{
		if (!init_gyro.sensorCalibrated())
		{
			init_gyro.setValuesforAverage(measurement[0], measurement[1], measurement[2]);
			std::cout << "number Gyr = " << gyr_count << std::endl;
			std::cout << "number Acc = " << acc_count << std::endl;
			std::cout << "number Mag = " << mag_count << std::endl;
			gyr_count++;
		}
		else
		{
			if (!Gyr_initialized)
			{
				double avg[3];
				double variance[3];
				init_gyro.getAverageValues(avg);
				init_gyro.getVariance(variance);
				std::copy(std::begin(avg), std::end(avg), avg_gyr.begin());
				std::copy(std::begin(variance), std::end(variance), variance_gyr.begin());
				Gyr_initialized = true;

				std::cout << "gyro average set" << std::endl;
				printf("average is [%f, %f, %f]", avg[0], avg[1], avg[2]);
				printf("variance is [%f, %f, %f]", variance[0], variance[1], variance[2]);

				std::string text = "Average gyro = [" + std::to_string(avg[0]) + "," + std::to_string(avg[1]) + "," + std::to_string(avg[2]) + "]\n";
				WriteTextFile(text);
				text = "Variance gyro = [" + std::to_string(variance[0]) + "," + std::to_string(variance[1]) + "," + std::to_string(variance[2]) + "]\n";
				WriteTextFile(text);
			}
		}
	}
}

void Parser::WriteCalibrationMeasurement(std::array<double, 3> measurement)
{
	Mag_calib.setValues(measurement[0], measurement[1], measurement[2]);
}
void Parser::WriteKalmanFilterMeasurement(std::array<double, 3> measurement, long long Time, char Type)
{
	/*
	0 - Acc, 1 - gyr, 2- mag
	*/

	if (gyro_is_set == false)
	{
		if (Type == '0')
		{
			setAcc0(measurement, Time);
			//std::cout << "after acc0" << std::endl;
		}

		else if (Type == '2')
		{
			setMag0(measurement, Time);
			//std::cout << "after mag0" << std::endl;
		}
		else
		{
			setGyr(measurement, Time);
			gyro_is_set = true;
			//std::cout << "gyro set" << std::endl;
		}
	}

	else
	{
		if (Type == '0')
		{
			setAcc1(measurement, Time);
			acc_1_is_set = true;
			//std::cout << "gyro set acc1" << std::endl;
		}

		else if (Type == '2')
		{
			setMag1(measurement, Time);
			mag_1_is_set = true;
			//std::cout << "gyro set mag1" << std::endl;
		}
		else
		{
			setGyr(measurement, Time);
			if (acc_1_is_set)
			{
				setAcc0(acc_1, time_acc_1);
				//std::cout << "gyro reset acc1 set acc0" << std::endl;
			}

			if (mag_1_is_set)
			{
				setMag0(mag_1, time_mag_1);
				//std::cout << "gyro reset mag_1 set mag0" << std::endl;

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

		ExecuteKalmanFilter();

		setAcc0(acc_1, time_acc_1);

		setMag0(mag_1, time_mag_1);

		//setGyr(std::array<double,3>(), Time);
	}

}

void Parser::NormalizeValues(double& x, double& y, double& z)
{
	double denom = (y) * (y)+(z) * (z)+(x) * (x);
	denom = sqrt(denom);
	x = x / denom; y = y / denom; z = z / denom;

}
void Parser::ExecuteKalmanFilter()
{
	std::array<double, 3> acc_interpolated = LinearInterpolationSensor(time_acc_0, time_acc_1, time_gyro, acc_0, acc_1);
	

	std::array<double, 3> mag_interpolated = LinearInterpolationSensor(time_mag_0, time_mag_1, time_gyro, mag_0, mag_1);
	std::string text;
	text = "acc_0 = [" + std::to_string(acc_0[0]) + " , " + std::to_string(acc_0[1]) + " , " + std::to_string(acc_0[2]) + "]\n";
	WriteTextFile(text);
	text = "acc_1 = [" + std::to_string(acc_1[0]) + " , " + std::to_string(acc_1[1]) + " , " + std::to_string(acc_1[2]) + "]\n";
	WriteTextFile(text);
	text = "acc interpolated = [" + std::to_string(acc_interpolated[0]) + " , " + std::to_string(acc_interpolated[1]) + " , " + std::to_string(acc_interpolated[2]) + "]\n";
	WriteTextFile(text);

	
	text = "mag_0 = [" + std::to_string(mag_0[0]) + " , " + std::to_string(mag_0[1]) + " , " + std::to_string(mag_0[2]) + "]\n";
	WriteTextFile(text);
	text = "mag_1 = [" + std::to_string(mag_1[0]) + " , " + std::to_string(mag_1[1]) + " , " + std::to_string(mag_1[2]) + "]\n";
	WriteTextFile(text);
	text = "mag interpolated = [" + std::to_string(mag_interpolated[0]) + " , " + std::to_string(mag_interpolated[1]) + " , " + std::to_string(mag_interpolated[2]) + "]\n";
	WriteTextFile(text);
	kalmanFilter.SetAngularVelocity(gyro[0], gyro[1], gyro[2], time_gyro);

	Mag_calib.CorrectValues(mag_interpolated[0], mag_interpolated[1], mag_interpolated[2]);

	NormalizeValues(mag_interpolated[0], mag_interpolated[1], mag_interpolated[2]);
	
	text = "mag interpolated normalized = [" + std::to_string(mag_interpolated[0]) + " , " + std::to_string(mag_interpolated[1]) + " , " + std::to_string(mag_interpolated[2]) + "]\n";
	WriteTextFile(text);

	kalmanFilter.SetMagnetometerMeasurements(mag_interpolated[0], mag_interpolated[1], mag_interpolated[2]);

	NormalizeValues(acc_interpolated[0], acc_interpolated[1], acc_interpolated[2]);
	
	text = "acc interpolated normalized = [" + std::to_string(acc_interpolated[0]) + " , " + std::to_string(acc_interpolated[1]) + " , " + std::to_string(acc_interpolated[2]) + "]\n";
	WriteTextFile(text);

	kalmanFilter.SetAccelerometerMeasurements(acc_interpolated[0], acc_interpolated[1], acc_interpolated[2]);

	double Quarternion[4];
	double RPY[3];

	kalmanFilter.getXk_1(Quarternion);
	kalmanFilter.getRPY(RPY);

	text = "roll pitch yaw =  [" + std::to_string(RPY[0]) + " , " + std::to_string(RPY[1]) + " , " + std::to_string(RPY[2]) + "]\n";
	WriteTextFile(text);

	plotter.setMVPmatrix(Quarternion);
}

std::array<double, 3> Parser::LinearInterpolationSensor(long long t1, long long t2, long long t3, std::array<double, 3> y1, std::array<double, 3> y2)
{
	std::array<double, 3> interpolatedValues;
	for (int i = 0; i < 3; ++i)
	{
		interpolatedValues[i] = (y2[i] - y1[i]) / ((double)t2 - (double)t1) * ((double)t3 - (double)t1) + y1[i];
	}
	return interpolatedValues;
}
void Parser::setAcc0(std::array<double, 3> sensor, long long Time)
{
	acc_0 = sensor;
	time_acc_0 = Time;
}

void Parser::setAcc1(std::array<double, 3> sensor, long long Time)
{
	acc_1 = sensor;
	time_acc_1 = Time;
}
void Parser::setMag0(std::array<double, 3> sensor, long long Time)
{
	mag_0 = sensor;
	time_mag_0 = Time;
}
void Parser::setMag1(std::array<double, 3> sensor, long long Time)
{
	mag_1 = sensor;
	time_mag_1 = Time;
}
void Parser::setGyr(std::array<double, 3> sensor, long long Time)
{
	gyro = sensor;
	time_gyro = Time;
}


std::string Parser::FindValues(std::string& str, std::string regex)
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
void Parser::WriteTextFile(std::string text)
{
	fout.open(file_source, std::ios::out | std::ios::app);
	fout << text << std::endl;
	fout.close();
}
void Parser::WriteCSV(std::array<double, 3> measurement, long long time, char sensorType)
{
	std::call_once(flag_, [&]() {
		fout.open(file_source, std::ios::out | std::ios::app);
		fout << "Sensor" << "," << "x" << "," << "y" << "," << "z" << "," << "time_stamp" << std::endl;
		});
	fout << sensorType << "," << measurement[0] << "," << measurement[1] << "," << measurement[2] << "," << time << std::endl;
}

void Parser::CreateAndRunOpenGLPlotter()
{
	plotter.Init();
	plotter.run();
}
void Parser::run()
{

	cv = s->getCV(); // pointer to condition variable
	bool onlyonce = true;
	std::mutex m2;
	double quart[] = { 1.0,0.0,0.0,0.0 };
	plotter.setMVPmatrix(quart);
	std::thread plotter_thread(&Parser::CreateAndRunOpenGLPlotter, this);


	while (true)
	{
		std::unique_lock<std::mutex> locker(m2); // access mutex by reference

		cv->wait(locker, [&]() {return !s->getSensorReadings().empty() ? true : false; });

		std::vector<std::string> readings = s->getSensorReadings();

		s->clearSensorReadings();

		locker.unlock();

		for (std::string Measurement : readings)
		{
			if (Measurement[0] == '#')
			{
				Measurement = Measurement.substr(1, Measurement.size() - 1);
				ProcessString(Measurement);
			}
		}
		onlyonce = false;
		if (s->getLoop() == false)
		{
			break;
		}
	}

	plotter_thread.join();

	std::cout << "out of loop" << std::endl;

}

void Parser::setMutex(std::mutex* m)
{
	m1 = m;
	std::cout << "mutex set in parser" << std::endl;
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
	std::cout << "server set in parser" << std::endl;
}

