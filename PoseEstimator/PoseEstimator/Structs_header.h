#pragma once

struct SensorReading
{
	char type;
	double x;
	double y;
	double z;
	long time_stamp;
};

struct LatestMeasurements
{
	double gyro[3];
	double acc[3];
	double mag[3];
	long time_stamp;
};

struct LeastSquares
{
	double* ATA;
	double* ATb;
	int numstates;
};




