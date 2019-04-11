#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <iostream>
#include <curand_mtgp32_kernel.h>
#include "cuda_runtime.h"
#include "cuda_headers.h"
#include <curand.h>
#include <vector>

__global__ void kernel(int n, float *x, float *y) {
  int index = threadIdx.x;
  int stride = blockDim.x;
  for (int i = index; i < n; i += stride) {
    y[i] = x[i] + y[i];
  }
}

void cudamain() {

  dev_array

  curandGenerator_t curand_generator;
  curandCreateGenerator(&curand_generator, CURAND_RNG_PSEUDO_MTGP32);
  curandSetPseudoRandomGeneratorSeed(curand_generator, 1234ULL);
  curandGenerateNormal(curand_generator, d)


  kernel <<<1,256>>>(N, x, y);

  cudaDeviceSynchronize();

  for (int i = 0; i < 10; i++) {
    std::cout << "x: " << x[i] << ", y: " << y[i] << std::endl;
  }

  cudaFree(x);
  cudaFree(y);
}
