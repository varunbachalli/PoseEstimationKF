// Example setup for CUDA
#include "CudaAbstraction.cuh"


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
