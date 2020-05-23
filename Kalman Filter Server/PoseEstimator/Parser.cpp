#include "Parser.h"

Parser::Parser()
{
	init_acc = InitialValues(100);
	init_mag = InitialValues(100);
	init_gyro = InitialValues(100);
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

		switch (phase)
		{
		case '1':
			if (!Mag_calib.MagnetometerCalibrated())
			{
				WriteCalibrationMeasurement(sensor_reading);
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

				Values["mag0"] = avg_mag;
				Values["acc0"] = avg_acc;
				NormalizeValues(Values["mag0"]);
				NormalizeValues(Values["acc0"]);
				kalmanFilter = KalmanFilter(Values, Time);

				setAcc0(avg_acc, Time);
				setMag0(avg_mag, Time);

				Kalman_initialized = true;
			}

			else
			{
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
				avg_acc = init_acc.getAverageValues();
				variance_acc = init_acc.getVariance();
				Acc_initialized = true;
			}
		}
	}	
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
				avg_mag = init_mag.getAverageValues();
				variance_mag = init_mag.getVariance();
				Mag_initialized = true;
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
				avg_gyr = init_gyro.getAverageValues();
				variance_gyr = init_gyro.getVariance();
				Gyr_initialized = true;
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
	if (gyro_is_set == false)
	{
		if (Type == '0')
		{
			setAcc0(measurement, Time);
			
		}

		else if (Type == '2')
		{
			setMag0(measurement, Time);
			
		}
		else if (Type == '1')
		{
			setGyr(measurement, Time);
			gyro_is_set = true;
			
		}
	}

	else
	{
		if (Type == '0')
		{
			setAcc1(measurement, Time);
			acc_1_is_set = true;
			
		}

		else if (Type == '2')
		{
			setMag1(measurement, Time);
			mag_1_is_set = true;
	
		}
		else if(Type == '1')
		{
			setGyr(measurement, Time);
			if (acc_1_is_set)
			{
				setAcc0(acc_1, time_acc_1);
				
			}

			if (mag_1_is_set)
			{
				setMag0(mag_1, time_mag_1);

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

	}

}

void Parser::NormalizeValues(std::array<double, 3>& arr)
{
	double denom = (arr[0] * arr[0]) + (arr[1] * arr[1]) + (arr[2] * arr[2]);
	denom = sqrt(denom);
	arr[0] = arr[0] / denom; 
	arr[1] = arr[1] / denom; 
	arr[2] = arr[2] / denom;
}
void Parser::ExecuteKalmanFilter()
{
	
	std::array<double, 3> acc_interpolated = LinearInterpolationSensor(time_acc_0, time_acc_1, time_gyro, acc_0, acc_1);
	

	std::array<double, 3> mag_interpolated = LinearInterpolationSensor(time_mag_0, time_mag_1, time_gyro, mag_0, mag_1);

	kalmanFilter.SetAngularVelocity(gyro[0], gyro[1], gyro[2], time_gyro);

	Mag_calib.CorrectValues(mag_interpolated[0], mag_interpolated[1], mag_interpolated[2]);

	NormalizeValues(mag_interpolated);
	NormalizeValues(acc_interpolated);

	kalmanFilter.SetMagnetometerMeasurements(mag_interpolated[0], mag_interpolated[1], mag_interpolated[2]);



	kalmanFilter.SetAccelerometerMeasurements(acc_interpolated[0], acc_interpolated[1], acc_interpolated[2]);

	double Quarternion[4];
	double RPY[3];

	kalmanFilter.getXk_1(Quarternion);
	kalmanFilter.getRPY(RPY);

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

