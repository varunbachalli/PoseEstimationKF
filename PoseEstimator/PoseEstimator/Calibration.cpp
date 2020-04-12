#include "Calibration.h"

void Calibration::setValues(double x_, double y_, double z_)
{
	*(x + num_rows) = x_;
	*(y + num_rows) = y_;
	*(z + num_rows) = z_;

	// pointer to row n = UncalibratedValues + n*rows

	num_rows++; // once it comes to 999 , then update to 1000,
	if (num_rows == total_values) 
	{
		bias(0) = bias(0) / total_values;
		bias(1) = bias(1) / total_values;
		bias(2) = bias(2) / total_values;
		values_set = true;
	}

	else if(num_rows < total_values)
	{
		bias(0) += x_;
		bias(1) += y_;
		bias(2) += z_;
	}

}

Eigen::Matrix3d Calibration::get_W()
{
	if (W.isZero())
	{
		setW();
	}
	return W;
}

Eigen::Vector3d Calibration::get_bias() // don't ask for it untill it's set
{
	return bias;
}



void Calibration::setUncalibratedValues()
{
	for (int i = 0; i < total_values; ++i)
	{
		double x_ = *(x + i) - bias(0); 
		double y_ = *(y + i) - bias(1);
		double z_ = *(z + i) - bias(2);
		if (i == 0)
		{
			printf("bias is [%f,%f,%f]", bias(0), bias(1), bias(2));
			printf("variables before [%f,%f,%f]", *(x + i), *(y + i), *(z + i));
			printf("variables after [%f,%f,%f]", x_,y_,z_);
			
		}
		*(UncalibratedValues + i ) = x_ * x_; // a
		*(UncalibratedValues + i  + total_values * 1) = y_ * y_; // b
		*(UncalibratedValues + i  + total_values * 2) = z_ * z_; // c
		*(UncalibratedValues + i  + total_values * 3) = 2 * x_ * y_; // d
		*(UncalibratedValues + i  + total_values * 4) = 2 * z_ * x_; // e
		*(UncalibratedValues + i  + total_values * 5) = 2 * y_ * z_; // f
	}

}

void Calibration::LargestOffDiagonal(Eigen::Matrix3d A, int a[2])
{
	int k = 0;
	int l = 0;
	double Max = std::numeric_limits<double>::min();

	for (int i = 0; i < A.rows(); ++i)
	{
		for (int j = i + 1; j < A.cols(); ++j)
		{
			if (A(i, j) > Max)
			{
				Max = A(i, j);
				k = i;
				l = j;
			}
		}
	}

	a[0] = k;
	a[1] = l;
}

bool Calibration::IsDoubleZero(double a)
{
	double eps = 0.0001;
	bool is_zero = false;
	if (a <= eps && a >= -1 * eps)
	{
		is_zero = true;
	}
	return is_zero;
}
Eigen_Vec_Vals Calibration::JacobiMethod(Eigen::Matrix3d A)
{
	Eigen_Vec_Vals Eig; 
	Eig.EigenValues = A; // D
	Eig.EigenVectors = Eigen::MatrixXd::Identity(A.rows(),A.cols());// S

	while (true)
	{
		int a[2];
		LargestOffDiagonal(Eig.EigenValues, a);
		
		double theta = 0.0;
		if(IsDoubleZero(Eig.EigenValues(a[0], a[0]) - Eig.EigenValues(a[1], a[1]))) // if (Eig.EigenValues[a][a] = Eig.EigenValues[b][b])
		{
			if (Eig.EigenValues(a[0], a[0]) > 0)
				theta = pi / 4;
			else
				theta = -pi / 4;
		}
		else
		{
			theta = 0.5 * atan2(2 * Eig.EigenValues(a[0], a[1]), Eig.EigenValues(a[0], a[0]) - Eig.EigenValues(a[1], a[1]));
		}
			
	

		if(IsDoubleZero(cos(theta) - 1)) // if cos(theta) == 1
			break;
		Eigen::Matrix3d S1 = Eigen::Matrix3d::Identity();
		S1(a[0], a[0]) =	cos(theta);
		S1(a[1], a[1]) =	cos(theta);
		S1(a[0], a[1]) = -1 * sin(theta);
		S1(a[1], a[0]) =	sin(theta);
		
		Eig.EigenValues = S1.transpose() * (Eig.EigenValues * S1);
		Eig.EigenVectors = Eig.EigenVectors * S1;
	}

	Eig.EigenValues = Eig.EigenValues.cwiseProduct(Eigen::Matrix3d::Identity());
	return Eig;
}

Calibration::Calibration()
{
	UncalibratedValues = (double*)malloc(sizeof(double) * total_values * 6);  //*(UncalibratedValues + i * col + j)
	x = (double*)malloc(sizeof(double) * total_values);
	y = (double*)malloc(sizeof(double) * total_values);
	z = (double*)malloc(sizeof(double) * total_values);
}

Calibration::~Calibration()
{
	free(x);
	free(y);
	free(z);
	free(UncalibratedValues);
}



Eigen::Matrix3d Calibration::LeastSquares_calculation(double* A)
{
	LeastSquares values = MatrixTransposeMultiplication(A, 6, total_values);
	Eigen::Map<Eigen::MatrixXd> ATA(values.ATA, 6, 6);
	Eigen::Map<Eigen::VectorXd> ATb(values.ATb, 6);
	Eigen::VectorXd X = ATA.colPivHouseholderQr().solve(ATb);
	Eigen::Matrix3d R;
	// [a,b,c,d,e,f] = X(0,1,2,3,4,5) needs to be transformed into
	// [[a d e]
	//	[d b f]
	//	[e f c]]
	// 
	
	R(0, 0) = X(0);
	R(1, 1) = X(1);
	R(2, 2) = X(2);
	R(0, 1) = X(3);
	R(1, 0) = X(3);
	R(0, 2) = X(4);
	R(2, 0) = X(4);
	R(1, 2) = X(5);
	R(2, 1) = X(5);

	
	return R;
}




void Calibration::setW()
{
	setUncalibratedValues();
	Eigen::Matrix3d R = LeastSquares_calculation(UncalibratedValues);
	std::cout << " R is \n" << R << std::endl;
	Eigen_Vec_Vals Eig = JacobiMethod(R);

	std::cout << "EigenValues are \n" << Eig.EigenValues << std::endl;
	std::cout << "EigenVectors are \n" << Eig.EigenVectors << std::endl;

	for (int i = 0; i < 3; ++i)
	{
		Eig.EigenValues(i, i) = sqrt(Eig.EigenValues(i, i));
	}
	W = (Eig.EigenVectors * Eig.EigenValues) * Eig.EigenVectors.transpose();
}

void print(double a[][6], int rows)
{
	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < 6; ++j)
		{
			std::cout << *(*(a + i) + j) << std::endl;
		}
	}

}

bool Calibration::SamplesToBeCollected()
{
	return values_set;
}


void TestCalibration()
{
	Calibration calib;
	std::ifstream file;
	file.open("D:/GITProjects/Kalman Filtering Server/PoseEstimationKF/data.csv");
	if (!file)
	{
		std::cerr << "couldn't open file" << std::endl;
	}
	else
	{
		for (int i = 0; i < 1000; ++i)
		{

			std::string outstring;
			file >> outstring;
			std::stringstream s_stream(outstring);
			double x[3];
			for (int j = 0; j < 3; ++j)
			{
				std::string substr;
				getline(s_stream, substr, ',');
				x[j] = std::stod(substr);
			}

			calib.setValues(x[0], x[1], x[2]);
		}
	}

	Eigen::Vector3d Bias = calib.get_bias();
	Eigen::Matrix3d W = calib.get_W();

}

int main()
{

	TestCalibration();
	_getch();
	return 0;
}