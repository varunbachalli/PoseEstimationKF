#include "Calibration.h"


// one kernel for multiplication

// one kernel for sum i = 1 to N (vector_1[i] * vector_2[i]) of vector 1 and 2

// one kernel for receiving UncalibratedValues matrix, and alloting the corresponding multiplication

void Calibration::setUncalibratedValues(double x_, double y_, double z_)
{
	*(x + num_rows) = x_;
	*(y + num_rows) = y_;
	*(z + num_rows) = z_;


	// pointer to row n = UncalibratedValues + n*rows


	num_rows++; // once it comes to 999 , then update to 1000,
	if (num_rows == total_values) 
	{
		bias(0, 0) = bias(0, 0) / total_values;
		bias(1, 0) = bias(1, 0) / total_values;
		bias(2, 0) = bias(2, 0) / total_values;
		values_set = true;
	}

	else if(num_rows < total_values)
	{
		bias(0, 0) += x_;
		bias(1, 0) += y_;
		bias(2, 0) += z_;
	}

}

Eigen::Matrix<double, 3, 3> Calibration::return_W()
{
	if (W.isZero())
	{
		setW();
	}
	return W;
}

Eigen::Matrix<double, 3, 1> Calibration::return_bias() // don't ask for it untill it's set
{
	return bias;
}



void Calibration::setUncalibratedValues()
{
	/*
	theta = np.random.uniform(0, np.pi, number_of_elements) 
    phi = np.random.uniform(0, 2*np.pi, number_of_elements)
    x = elips_equation[0] * np.multiply(np.sin(theta), np.cos(phi)) 
    y = elips_equation[1] * np.multiply(np.sin(theta), np.sin(phi)) 
    z = elips_equation[2] * np.cos(theta)
	*/
	/*srand(0);
	double pi = 3.14159265358979323846;
	double elips[] = { 2,0,3.0,5.0 };
	for (int i = 0; i < total_values; ++i)
	{
		double theta = (rand() % 180)* pi/180.0;
		double phi = (rand() % 360) * pi / 180.0;
		UncalibratedValues(i, 0) = elips[0] * sin(theta) * cos(phi);
		UncalibratedValues(i, 1) = elips[1] * sin(theta) * sin(phi);
		UncalibratedValues(i, 2) = elips[2] * cos(theta);
	}

	printf("UncalibratedValues has %d rows and %d columns\n", (int)UncalibratedValues.rows(), (int)UncalibratedValues.cols());*/

	for (int i = 0; i < total_values; ++i)
	{
		double x_ = *(x + i) - bias[0]; 
		double y_ = *(y + i) - bias[1];
		double z_ = *(z + i) - bias[2];
		*(UncalibratedValues + i ) = x_ * x_; // a
		*(UncalibratedValues + i  + rows * 1) = y_ * y_; // b
		*(UncalibratedValues + i  + rows * 2) = z_ * z_; // c
		*(UncalibratedValues + i  + rows * 3) = 2 * x_ * y_; // d
		*(UncalibratedValues + i  + rows * 4) = 2 * z_ * x_; // e
		*(UncalibratedValues + i  + rows * 5) = 2 * y_ * z_; // f
	}

}

Calibration::Calibration()
{
	UncalibratedValues = (double*)malloc(sizeof(double) * rows * cols);  //*(UncalibratedValues + i * col + j)
	x = (double*)malloc(sizeof(double) * rows);
	y = (double*)malloc(sizeof(double) * rows);
	z = (double*)malloc(sizeof(double) * rows);
	/*
	for (i = 0; i < row; i++)
      for (j = 0; j < col; j++)
         *(arr + i*col + j) = i + j;
	*/
}

Calibration::~Calibration()
{
	free(UncalibratedValues);
}



void Calibration::returnA_Atranspose(double A[][6], double A_[6][6])
{
	// A_ = A'. A
}




void Calibration::returnAtranspose_b(double A[][6], double b[6], int size)
{
	for (int i = 0; i < 6; i++)
	{
		std::cout << "[\t";
		for (int k = 0; k < size; ++k)
		{
			b[i] += A[k][i];
			std::cout << A[k][i] << " + \t";
		}
		std::cout <<" = " << b[i] << std::endl;
	}
}

void Calibration::setW()
{

	//Eigen::Matrix<double, total_values, 6> A;
	//for (int i = 0; i < total_values; ++i)
	//{
	//	A(i, 0) = UncalibratedValues(i, 0) * UncalibratedValues(i, 0); // a
	//	A(i ,1) = UncalibratedValues(i, 1) * UncalibratedValues(i, 1); // b
	//	A(i, 2) = UncalibratedValues(i, 2) * UncalibratedValues(i, 2); // c
	//	A(i, 3)	 = 2 * UncalibratedValues(i, 0) * UncalibratedValues(i, 1); // d
	//	A(i, 4)	 = 2 * UncalibratedValues(i, 2) * UncalibratedValues(i, 0); // e
	//	A(i, 5)	 = 2 * UncalibratedValues(i, 2) * UncalibratedValues(i, 1); // f
	//}
	//Eigen::Matrix<double, 6, 6> A_transp = A.transpose();
	//Eigen::VectorXd ones = Eigen::VectorXd::LinSpaced(total_values, 1.0, 1.0);
	//Eigen::VectorXd b = A_transp * ones;
	//Eigen::Matrix<double, 6, 6> A_ = A_transp * A;


	//Eigen::Matrix<double, 6, 1> R = A_.colPivHouseholderQr().solve(b);
	//printf("A has %d rows and %d columns\n", (int)A.rows(), (int)A.cols());
	//printf("b has %d rows and %d columns\n", (int)b.rows(), (int)b.cols());
	//std::cout << "R is " << std::endl;
	//printf("[%f,\t %f,\t %f,\t %f,\t %f,\t %f]\n", R(0), R(1), R(2), R(3), R(4), R(5));
	//std::cout << "R should be" << std::endl;
	//printf("[%f,\t %f,\t %f,\t %f,\t %f,\t %f]\n", 4.0 , 9.0 , 25.0 ,0.0 ,0.0 ,0.0);

}

void Calibration::Set_R_LeastSquares(Eigen::Matrix<double, Calibration::total_values, 6>& A)
{
	
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

//int main()
//{
//	Calibration c;
//	double A[][6] = {{1.0,1.0,1.0,1.0,1.0,1.0},
//					{1.0,1.0,1.0,1.0,1.0,1.0},
//					{1.0,1.0,1.0,1.0,1.0,1.0},
//					{1.0,1.0,1.0,1.0,1.0,1.0},
//					{1.0,1.0,1.0,1.0,1.0,1.0},
//					{1.0,1.0,1.0,1.0,1.0,1.0}};
//	double b[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
//	c.returnAtranspose_b(A, b ,6);
//
//	std::cout << "printing A" << std::endl;
//
//	print(A, 6);
//
//
//	std::cout << "printing b" << std::endl;
//	for (int i = 0; i < 6; ++i)
//	{
//		std::cout << b[i] << std::endl;
//	}
//
//	int k;
//	std::cin >> k;
//}


