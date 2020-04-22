#include "Parser.h"


// #### 
// IMPORTANT !!! 
// ####

// tasks to do .. 
// create the opengl platform.
// follow all function calls once
// Run Unit Test for each Class
// run the whole thing


Parser::Parser()
{
	init_acc = InitialValues(300);
	init_mag = InitialValues(300);
	init_gyro = InitialValues(300);

}

void Parser::ProcessString(std::string& str)
{
	phase = FindValues(str, ",")[0];
	std::array<double, 3> sensor_reading;
	long long Time;
	char Type;
	Type = FindValues(str, ":")[0];
	sensor_reading[0] = std::stod(FindValues(str, ","));
	sensor_reading[1] = std::stod(FindValues(str, ","));
	sensor_reading[2] = std::stod(FindValues(str, ","));
	Time = std::stol(FindValues(str, "t:"));
	std::thread csvThread(&Parser::WriteCSV, this, sensor_reading, Time, Type);
	switch (phase)
	{
	case '1':
		if (!Mag_calib.MagnetometerCalibrated())
			WriteCalibrationMeasurement(sensor_reading);
		break;
	case '2':
		if (valuesInitDone < 3)
			initialMeanAndCovariance(sensor_reading, Type);
		if (valuesInitDone == 3)
		{
			std::map<std::string, std::array<double, 3>> Values;
			Values["mag0"] = avg_mag;
			Values["acc0"] = avg_acc;
			Values["mag_sig"] = variance_mag;
			Values["acc_sig"] = variance_acc;
			Values["gyro_drift_0"] = avg_gyr;
			Values["gyro_drift_sig"] = variance_gyr;
			kalmanFilter = KalmanFilter(Values, Time);
			setAcc0(avg_acc, Time);
			setMag0(avg_mag, Time);
			valuesInitDone++;
		}
		if (valuesInitDone > 3)
		{
			setAcc0(avg_acc, Time);
			setMag0(avg_mag, Time);
			kalmanFilter.UpdateLatestPreviousTime(Time);
		}
		break;

	case '3':
		WriteKalmanFilterMeasurement(sensor_reading, Time, Type); // Change the Way the data is read.. Seems wrong now.
		break;
	default:
		break;
	}
	csvThread.join();
}

void Parser::initialMeanAndCovariance(std::array<double,3> measurement, char type)
{
	if (type == '0')
	{
		if (init_acc.sensorCalibrated())
		{
			init_acc.setValuesforAverage(measurement[0], measurement[1], measurement[2]);
		}
		else
		{
			double avg[3];
			double variance[3];
			init_acc.getAverageValues(avg);
			init_acc.getVariance(variance);
			std::copy(std::begin(avg), std::end(avg), avg_acc.begin());
			std::copy(std::begin(variance), std::end(variance), variance_acc.begin());
			valuesInitDone++;
		}
	}	// set values std::copy(std::begin(arr), std::end(arr), first.begin());
	else if (type == '2')
	{
		if (init_mag.sensorCalibrated())
		{
			Mag_calib.CorrectValues(measurement[0], measurement[1], measurement[2]);
			init_mag.setValuesforAverage(measurement[0], measurement[1], measurement[2]);
		}
		else
		{
			double avg[3];
			double variance[3];
			init_mag.getAverageValues(avg);
			init_mag.getVariance(variance);
			std::copy(std::begin(avg), std::end(avg), avg_mag.begin());
			std::copy(std::begin(variance), std::end(variance), variance_mag.begin());
			valuesInitDone++;
		}

	}
	if (type == '1')
	{
		if (init_gyro.sensorCalibrated())
		{
			init_gyro.setValuesforAverage(measurement[0], measurement[1], measurement[2]);
		}
		else
		{
			double avg[3];
			double variance[3];
			init_gyro.getAverageValues(avg);
			init_gyro.getVariance(variance);
			std::copy(std::begin(avg), std::end(avg), avg_gyr.begin());
			std::copy(std::begin(variance), std::end(variance), variance_gyr.begin());
			valuesInitDone++;
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
			std::cout << "after acc0" << std::endl;
		}

		else if (Type == '2')
		{
			setMag0(measurement, Time);
			std::cout << "after mag0" << std::endl;
		}
		else
		{
			setGyr(measurement, Time);
			gyro_is_set = true;
			std::cout << "gyro set" << std::endl;
		}
	}

	else
	{
		if (Type == '0')
		{
			setAcc1(measurement, Time);
			acc_1_is_set = true;
			std::cout << "gyro set acc1" << std::endl;
		}

		else if (Type == '2')
		{
			setMag1(measurement, Time);
			mag_1_is_set = true;
			std::cout << "gyro set mag1" << std::endl;
		}
		else
		{
			setGyr(measurement, Time);
			if (acc_1_is_set)
			{
				setAcc0(acc_1,time_acc_1);
				std::cout << "gyro reset acc1 set acc0" << std::endl;
			}

			if (mag_1_is_set)
			{
				setMag0(mag_1, time_mag_1);
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

		ExecuteKalmanFilter();
		setAcc0(acc_1, time_acc_1);
		setMag0(mag_1, time_mag_1);
		setGyr(std::array<double,3>(), Time);
	}

}

void Parser::NormalizeValues(double* x, double* y, double* z)
{
	double denom = (*y) * (*y) + (*z) * (*z) + (*x) * (*x);
	denom = sqrt(denom);
	*x = *x / denom;*y = *y / denom;*z = *z / denom;

}
void Parser::ExecuteKalmanFilter()
{
	std::array<double, 3> acc_interpolated = LinearInterpolationSensor(time_acc_0, time_acc_1, time_gyro, acc_0, acc_1);
	std::array<double, 3> mag_interpolated = LinearInterpolationSensor(time_mag_0, time_mag_1, time_gyro, mag_0, mag_1);
	kalmanFilter.SetAngularVelocity(gyro[0], gyro[1], gyro[2], time_gyro);
	Mag_calib.CorrectValues(mag_interpolated[0], mag_interpolated[1], mag_interpolated[2]);
	kalmanFilter.SetMagnetometerMeasurements(mag_interpolated[0], mag_interpolated[1], mag_interpolated[2]);
	NormalizeValues(&acc_interpolated[0], &acc_interpolated[1], &acc_interpolated[2]);
	kalmanFilter.SetAccelerometerMeasurements(acc_interpolated[0], acc_interpolated[1], acc_interpolated[2]);
	double Quarternion[4];
	double RPY[3];
	kalmanFilter.getXk_1(Quarternion);
	kalmanFilter.getRPY(RPY);
	


	/*

	send quarternion to opengl plotter

	*/
}

std::array<double,3> Parser::LinearInterpolationSensor(long long t1, long long t2, long long t3, std::array<double, 3> y1, std::array<double, 3> y2)
{
	std::array<double, 3> interpolatedValues;
	for (int i = 0; i < 3; ++i)
	{
		interpolatedValues[i]= ((y2[i] - y1[i]) / ((double)t2 - (double)t1))* ((double)t3) + y1[i];
	}
	return interpolatedValues;
}
void Parser::setAcc0(std::array<double,3> sensor, long long Time)
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

void Parser::WriteCSV(std::array<double, 3> measurement, long long time, char sensorType)
{
	std::call_once(flag_, [&]() {
		fout.open(file_source, std::ios::out | std::ios::app);
		fout << "Sensor" << "," << "x" << "," << "y" << "," << "z" << "," << "time_stamp" << std::endl;
		});
	fout << sensorType << "," << measurement[0] << "," << measurement[1] << "," << measurement[2] << "," << time << std::endl;
}

void Parser::run()
{
	cv = s->getCV(); // pointer to condition variable
	while (true)
	{
		std::unique_lock<std::mutex> locker(*m1); // access mutex by reference
		cv->wait(locker, [&]() {return !s->getSensorReadings().empty(); });
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

