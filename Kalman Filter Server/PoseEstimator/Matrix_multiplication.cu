#include "Matrix_multiplication.cuh"
#include <fstream>
#include <string>
#include <chrono>

__global__ void multiply(double* A, double* A_t, int numvalues)
{
	int id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id < numvalues)
	{
		*(A_t + id) = (*(A + id)) * (*(A_t + id));
	}
}


__global__ void ATA(double* A, double* A_t, double* C, int num_values_in_A) // A and A_t = numstates x numsamples 
{
	int id = blockIdx.x* blockDim.x + threadIdx.x;
	int num_samples = num_values_in_A / blockDim.x;
	if (id == 0)
	{
		multiply <<<num_values_in_A/512 + 1, 512>>>(A, A_t,num_values_in_A);
	}

	cudaDeviceSynchronize();
	__syncthreads();
	for (int i = 0; i < num_samples; ++i)
	{
		*(C + id) += *(A_t + id * num_samples + i);
	}
}

__global__ void ATb(double* A, double* b, int num_samples)
{
	int id = blockIdx.x * blockDim.x + threadIdx.x;
	b[id] = 0;
	for (int i = 0; i < num_samples; ++i)
	{
		b[id] += A[id * num_samples + i];
	}
}

int numThreads(int numstates)
{
	int k = 0;
	if ((numstates + 1) % 2 == 1)
	{
		k = (numstates + 1) / 2 + 1;
	}
	else
	{
		k = (numstates + 1) / 2;
	}

	return k;
}

double* PermutateA(double* A, int num_samples, int num_states, int num_shifts)
{
	double* A_perm = (double*)malloc(sizeof(double) * num_samples * num_states);

	int size_block1 = sizeof(double) * (num_samples) * (num_states - num_shifts); // chunk that goes to the front 
	int size_block2 = sizeof(double) * (num_samples) * (num_shifts); // chunk that goes to the back

	memcpy(A_perm, (A + (num_samples) * (num_shifts)), size_block1);
	memcpy(A_perm + (num_samples) * (num_states - num_shifts), A, size_block2);
	return A_perm;
}

void printValues(double* A, int numstates, int numsamples)
{
	for (int i = 0; i < numstates; ++i)
	{
		for (int j = 0; j < numsamples; ++j)
		{
			std::cout << *(A + i * numsamples + j) << " , ";

		}
		std::cout << std::endl;
	}
}


LeastSquares MatrixTransposeMultiplication(double* A, int numstates, int numsamples)
{
	int nStreams = numThreads(numstates);
	std::cout << "these many streams need to be made" << std::endl;
	std::cout << nStreams << std::endl;

	double* C = (double*)malloc(sizeof(double) * nStreams * numstates);
	double* b = (double*)malloc(sizeof(double) * numstates);
	std::vector<double*> pointersToA_perm_matrices;
	cudaStream_t* cudaStreams = new cudaStream_t[nStreams+1]; // nStreams for ATA, 1 stream for ATb
	
	for (int i = 0; i < nStreams; ++i) // create n streams
	{
		cudaStreamCreate(&cudaStreams[i]);
		pointersToA_perm_matrices.push_back(PermutateA(A, numsamples, numstates, i));
	}

	cudaStreamCreate(&cudaStreams[nStreams]); // create stream for ATb

	double* d_A;
	double* d_A_transpose;
	double* dC;
	double* db;
	cudaMalloc(&d_A, sizeof(double) * numstates * numsamples);
	cudaMalloc(&d_A_transpose, sizeof(double) * numstates * numsamples * nStreams);
	cudaMalloc(&dC, sizeof(double) * nStreams * numstates);
	cudaMalloc(&db, sizeof(double) * numstates);
	cudaMemcpy(d_A, A, sizeof(double) * numstates * numsamples,cudaMemcpyHostToDevice);
	
	int streamSize = numstates * numsamples;

	for (int i = 0; i < nStreams; ++i)
	{
		int offset = i * streamSize;
		cudaMemcpyAsync(d_A_transpose + offset, pointersToA_perm_matrices[i], sizeof(double) * numstates * numsamples, cudaMemcpyHostToDevice, cudaStreams[i]);
	}

	for (int i = 0; i < nStreams; ++i) {
		int offset = i * streamSize;
		ATA<<<1, numstates, 0, cudaStreams[i]>>> (d_A, d_A_transpose + offset, dC + i*numstates, numsamples*numstates);
	}

	ATb <<<1, numstates, 0, cudaStreams[nStreams] >>> (d_A, db, numsamples); // ATb

	for (int i = 0; i < nStreams; ++i) {
		cudaMemcpyAsync(C + i * numstates, dC + i * numstates, sizeof(double)*numstates, cudaMemcpyDeviceToHost, cudaStreams[i]);
	}
	cudaMemcpyAsync(b, db, sizeof(double) * numstates, cudaMemcpyDeviceToHost, cudaStreams[nStreams]);


	for (int i = 0; i < nStreams+1; ++i) {
		cudaStreamSynchronize(cudaStreams[i]);
	}
	
	/*std::cout << "above is c Printed " << std::endl;
	printValues((double*)C, nStreams, numstates);*/

	

	for (int i = 0; i < nStreams+1; ++i) 
	{
		cudaStreamDestroy(cudaStreams[i]);
	}
	
	LeastSquares Solution;
	Solution.ATA = new double[numstates * numstates];
	Solution.ATb = b;
	
	for (int i = 0; i < nStreams; ++i)
	{
		for (int k = 0; k < numstates; ++k)
		{
			if ((i + k) < numstates)
			{
				Solution.ATA[k * numstates + (i + k)] = C[numstates * i + k];
				Solution.ATA[(i + k) * numstates + k] = C[numstates * i + k];
			}
			else
			{
				Solution.ATA[numstates * (i + k - numstates) + k] = C[numstates * i + k];
				Solution.ATA[k * numstates + (i + k - numstates)] = C[numstates * i + k];
			}
		}
	}

	Solution.numstates = numstates;
	delete [] cudaStreams;
	cudaFree(d_A);
	cudaFree(d_A_transpose);
	cudaFree(dC);
	cudaFree(db);
	free(C);
	return Solution;
}

double* getA()
{
	double* A = (double*)malloc(6000 * sizeof(double));
	std::ifstream file;
	file.open("D:/GITProjects/Kalman Filtering Server/PoseEstimationKF/Sensor_CSV/myfile.txt");
	if (!file)
	{
		std::cerr << "couldn't open file" << std::endl;
	}
	else
	{
		for (int i = 0; i < 6000; ++i)
		{
			std::string outstring;
			file >> outstring;
			*(A + i) = std::stod(outstring);
		}
	}
	std::cout << "could read the file \n";

	return A;
}


//
//int main()
//{
//	double* A_ = getA();
//	int numberofDataPoints = 1000;
//	int numberofStates = 6;
//
//	//double A_[] = { 5, 7, 9, 9, 9, 0, 3, 5, 5, 7,
//	//				2, 7, 7, 2, 8, 4, 7, 0, 5, 0,
//	//				8, 4, 2, 3, 3, 9, 0, 5, 1, 6,
//	//				8, 2, 3, 2, 0, 7, 8, 8, 3, 8,
//	//				5, 5, 8, 9, 2, 4, 8, 3, 5, 5,
//	//				0, 4, 7, 4, 3, 5, 4, 8, 6, 3 };
//
//
//
//
//	LeastSquares Solution;
//
//	/*
//	Solution.ATA = Eigen::Map<Eigen::MatrixXd>(A_, numberofDataPoints, numberofStates);
//	Solution.ATb = Eigen::Map<Eigen::VectorXd>(A_, numberofDataPoints* numberofStates);
//	std::cout << "first" << std::endl;
//	std::cout << Solution.ATA << std::endl;
//	std::cout << "second" << std::endl;
//	std::cout << Solution.ATb << std::endl;
//	*/
//
//
//	/*printf("original matrix is");
//	printValues((double*)A_, numberofStates, numberofDataPoints);*/
//	auto start = std::chrono::high_resolution_clock::now();
//	Solution = MatrixTransposeMultiplication((double*)A_, numberofStates, numberofDataPoints);
//	std::cout << "Solution.ATA is" << std::endl;
//	printValues(Solution.ATA, numberofStates, numberofStates);
//
//	std::cout << "b is " << std::endl;
//	std::cout << "[";
//	for (int i = 0; i < numberofStates; ++i)
//	{
//		std::cout << Solution.ATb[i] << "\t";
//	}
//	std::cout << "]\n";
//
//	free(Solution.ATA);
//	free(Solution.ATb);
//	free(A_);
//
//	auto stop = std::chrono::high_resolution_clock::now();
//
//	auto duration = std::chrono::duration_cast <std::chrono::milliseconds> (stop - start);
//	std::cout << "time taken is " << duration.count() << std::endl;
//	getch();
//
//	return 0;
//
//	//double Answer[] = { 425 ,258 ,212 ,234 ,330, 257,
//	//					258 ,260 ,129 ,154 ,232, 187,
//	//					212 ,129 ,245 ,238 ,195, 160,
//	//					234 ,154 ,238 ,331 ,263, 210,
//	//					330 ,232 ,195 ,263 ,338, 239,
//	//					257 ,187 ,160 ,210 ,239, 240 };
//
//	//
//
//	//std::cout << "answer is supposed to be" << std::endl;
//	//printValues((double*)Answer, 6, 6);
//
//	//double b[] = { 59,42,41,49,54,44.0 };
//	//std::cout << "b is supposed to be " << std::endl;
//	//std::cout << "[";
//	//for (double k : b)
//	//{
//	//	std::cout << k << "\t,";
//	//}
//	//std::cout << "]\n";
//
//}