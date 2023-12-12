// STD Includes
#include <chrono>
#include <iostream>
#include <numeric>

// PCL Includes
#include <pcl/gpu/containers/device_array.h>

// 添加cuda库
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

// CUDA Helper Includes
#include "helper_cuda.h"

// Registration Includes
#include "L2_PE_gpu.cuh"

__global__ void calculateOnePointPotentialEnergy(pcl::gpu::PtrSz<pcl::PointXYZ> input,
                                                 pcl::gpu::PtrSz<pcl::PointXYZ> target,
                                                 thrust::device_ptr<float> sums, int input_size,
                                                 int target_size)
{
  float localSum = 0;
  int inputIndex = blockDim.x * blockIdx.x + threadIdx.x;
  if (inputIndex >= input_size) {
    return;
  }
  for (int i = 0; i < target_size; i++) {
    float dx = __powf(std::abs(input[inputIndex].x - target[i].x), 2);
    float dy = __powf(std::abs(input[inputIndex].y - target[i].y), 2);
    float dz = __powf(std::abs(input[inputIndex].z - target[i].z), 2);
    float dis = dx + dy + dz;
//    printf("%f \n", dx);
//    printf("%f \n", dy);
//    printf("%f \n", dz);
    if (dis < 0.0000001f) {
      dis = 0.0000001f;
    }
    localSum += -1.0f / dis;
  }
  sums[inputIndex] = localSum;
}

float cloud2GPU(pcl::gpu::DeviceArray<pcl::PointXYZ>& input,
                pcl::gpu::DeviceArray<pcl::PointXYZ>& target, float& fitnessScore,
                std::vector<int>& inputIndex, std::vector<int>& targetIndex)
{
  int n = input.size();
  thrust::host_vector<float> cpuSums(inputIndex.size());
  thrust::device_vector<float> gpuSums = cpuSums;

  int thread = 64;
  int block = (n + thread + 1) / thread;
  auto begin = std::chrono::high_resolution_clock::now();

  calculateOnePointPotentialEnergy<<<block, thread>>>(input, target, gpuSums.data(),
                                                      inputIndex.size(), targetIndex.size());

  cpuSums = gpuSums;
  fitnessScore = std::accumulate(cpuSums.cbegin(), cpuSums.cend(), 0.0f);
  checkCudaErrors(cudaDeviceSynchronize());
  return false;
}
