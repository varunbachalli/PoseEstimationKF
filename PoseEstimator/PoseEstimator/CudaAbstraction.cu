// Example setup for CUDA
#include "CudaAbstraction.cuh"
#include <stdio.h>
#include<stdlib.h>
#include<time.h>
#include <chrono> 
#include<vector>
#include <device_functions.h>
using namespace std::chrono;


__global__ void Test(int* a, int* b)
{
	*a += *b;
	// empty kernel
}


int RunCudaTest()
{
	int a = 5, b = 9;

	int* da, * db;

	cudaMalloc(&da, sizeof(int));
	cudaMalloc(&db, sizeof(int));

	cudaMemcpy(da, &a, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(db, &b, sizeof(int), cudaMemcpyHostToDevice);

	Test <<<1, 1 >>>(da, db);

	cudaMemcpy(&a, da, sizeof(int), cudaMemcpyDeviceToHost);
	std::cout << "answer is " << a << std::endl;

	cudaFree(da);
	cudaFree(db);
	return a;
}

void MatrixAddition(int* a, int* b, int* result, int m, int n)
{
	for (int i = 0; i < m; ++i)
	{
		for (int j = 0; j < n; ++j)
		{
			*((result + i*n) + j) = *((a + i * n) + j) + *((b + i * n) + j);
		}
		
	}
	
}

void print(int* arr, int m, int n)
{
	int i, j;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			printf("%d ", *((arr + i * n) + j));
		}
		std::cout << std::endl;
	}

}

__global__ void Add_Matrices(int* a, int* b, int* result, int max_threads)
{
	
	int id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id < max_threads)
	{
		*(result + id) = *(a + id) + *(b + id);
	}
}

__global__ void Multiply(int* a)
{
	int i = threadIdx.x + blockDim.x * blockIdx.x;
	if (i < 3)
	{
		a[i] = a[i + 1];
		
	}
	
}

int main()
{
	int count = 0;
	for(int l = 0; l < 10000; ++l)
	{	
		int a[] = { 1,2,3,4 };
		int* da;
		cudaMalloc(&da, sizeof(a));
		cudaMemcpy(da, a, sizeof(a), cudaMemcpyHostToDevice);
		Multiply <<<1, 5 >>> (da);
		cudaMemcpy(a, da, sizeof(a), cudaMemcpyDeviceToHost);
		if (a[0] != 2 && a[1] != 3 && a[2] != 4)
			count++;
		cudaFree(da);
	}

	std::cout << count << std::endl;
	//srand(time(0));
	//
	//std::vector<double> time_cpu;
	//std::vector<double> time_gpu;


	//for (int m = 100000; m <= 10000000; m+= 100000)
	//{
	//	/*std::cout << "vector size" << m << std::endl;*/
	//	int* xp = (int*)malloc(m * sizeof(int));
	//	int* yp = (int*)malloc(m * sizeof(int));
	//	int* result = (int*)malloc(m * sizeof(int));
	//	for (int i = 0; i < m; ++i)
	//	{
	//		*xp = rand() % 10000000;
	//		*yp = rand() % 10000000;
	//	}

	//	// time start gpu
	//	auto start = high_resolution_clock::now();
	//	int* da; int* db; int* dres; 

	//	cudaMalloc(&da, 4 * m);
	//	cudaMalloc(&db, 4 * m );
	//	cudaMalloc(&dres, 4 * m);
	//	
	//

	//	cudaMemcpy(da, xp, 4 * m, cudaMemcpyHostToDevice);
	//	cudaMemcpy(db, yp, 4 * m, cudaMemcpyHostToDevice);
	//	
	//	Add_Matrices <<<m / 1024 + 1, 1024>>> (da, db, dres, m);

	//	cudaMemcpy(result, dres, sizeof(int) * m, cudaMemcpyDeviceToHost);
	//	
	//	cudaFree(da);
	//	cudaFree(db);
	//	cudaFree(dres);
	//	// time end gpu
	//	auto stop = high_resolution_clock::now();
	//	double duration = (double)duration_cast<nanoseconds>(stop - start).count();
	//	time_gpu.push_back(duration);
	//	//std::cout << "gpu duration" << duration << std::endl;
	//	start= high_resolution_clock::now();
	//	// time start cpu

	//	for (int i = 0; i < m; ++i)
	//	{
	//		*(result + i) = *(xp + i) + *(yp + i);
	//	}

	//	// time end cpu

	//	free(xp);
	//	free(yp);
	//	free(result);
	//	stop = high_resolution_clock::now();
	//	duration = (double)duration_cast<nanoseconds>(stop - start).count();
	//	time_cpu.push_back(duration);
	//	//std::cout << "cpu duration" << duration << std::endl;
	//	
	//}
	//int gpu_faster = 0;
	//bool starting_first = false;
	//for (int i = 0; i < time_cpu.size(); ++i)
	//{
	//	if (time_gpu[i] - time_cpu[i] < 0.0)
	//	{
	//		gpu_faster++;
	//		if (!starting_first)
	//			std::cout << i << std::endl; starting_first = true;
	//	}

	//}

	//std::cout << "gpu is faster this many times :" << gpu_faster << std::endl;


	//int i = 100;
	//int l = 3;

	//int j = i / l;

	//std::cout << j << std::endl;


	//const int m = 2, n = 2;
	//int xp[m][n] = {{1,2} ,{1,2}};
	//int yp[m][n] = {{1,2} ,{1,2}};
	//int result[m][n];

	//int* da; int* db; int* dres;

	//cudaMalloc(&da, sizeof(int) * m * n);
	//cudaMalloc(&db, sizeof(int) * m * n);
	//cudaMalloc(&dres, sizeof(int) * m * n);

	//cudaMemcpy(da, (int*)&xp, sizeof(int) * m * n, cudaMemcpyHostToDevice);
	//cudaMemcpy(db, (int*)&yp, sizeof(int) * m * n, cudaMemcpyHostToDevice);

	//Add_Matrices <<<1, m*n>>> (da, db ,dres);

	//cudaMemcpy((int*)&result, dres, sizeof(int) * m * n, cudaMemcpyDeviceToHost);
	//print((int*)result, m, n);

	//cudaFree(da);
	//cudaFree(db);
	//cudaFree(dres);
	//std::cout << "Two dimensional Array stuff worked" << std::endl;
	int k;
	std::cin >> k;
	return 0;
}





//
//int main(int argc, char* argv[])
//{
//

//
//
//	MatrixAddition(&xp, &yp, &result, m, n);
//
//	
//
//	MatrixAddition(&xp, &xp);
//	std::cout << sizeof(xp) << std::endl;
//	std::cout << sizeof(int) * 10 * 22 << std::endl;
//	int k;
//	std::cin >> k;
//
//
//
//	return 0;	
//}


// use opengl to plot stuff

