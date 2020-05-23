#include <cuda_runtime.h>
#include <iostream>
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <conio.h>


struct LeastSquares
{
	double* ATA;
	double* ATb;
	int numstates;
};


__global__ void multiply(double* A, double* A_t, int numvalues);
__global__ void ATA(double* A, double* A_t, double* C, int num_values_in_A);// A and A_t = numstates x numsamples 
int numThreads(int numstates);
double* PermutateA(double* A, int num_samples, int num_states, int num_shifts);
LeastSquares MatrixTransposeMultiplication(double* A, int numstates, int numsamples);
double* getA();

