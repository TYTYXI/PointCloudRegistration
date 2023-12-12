//
// Created by XI on 2023/9/30.
//
// STD Includes
#include <chrono>
#include <utility>

// PCL Includes
#include <pcl/common/io.h>
#include <pcl/gpu/containers/device_array.h>

// utilities
#include "../utilities/PCR_Utility.h"

//
#include "L2_PE_gpu.cuh"
#include "L2_PE_Problem.h"

L2_PE_Problem::L2_PE_Problem()
{
}

L2_PE_Problem::~L2_PE_Problem()
{
}

oa::VectorDouble L2_PE_Problem::fitnessScore(const oa::VectorDouble& dv)
{
  TICK(a)
  std::uniform_int_distribution<int> input_rand(0, source_->size() - 1);
  std::uniform_int_distribution<int> target_rand(0, target_->size() - 1);

  std::random_device dev;
  std::mt19937 rng(dev());
  std::vector<int> input_index(256);
  std::vector<int> target_index(256);

  for (int i = 0; i < 256; i++) {
    input_index[i] = input_rand(rng);
    target_index[i] = target_rand(rng);
  }

  sort(input_index.begin(), input_index.end());
  sort(target_index.begin(), target_index.end());
  input_index.erase(unique(input_index.begin(), input_index.end()), input_index.end());
  target_index.erase(unique(target_index.begin(), target_index.end()), target_index.end());
  input_index.shrink_to_fit();
  target_index.shrink_to_fit();

  pcl::PointCloud<PointT> input;
  pcl::PointCloud<PointT> target;

  for (const auto& kv : std::as_const(input_index)) {
    input.emplace_back((*source_)[kv]);
  }
  for (const auto& kv : std::as_const(target_index)) {
    target.emplace_back((*target_)[kv]);
  }

  calculateTransPointCloud<PointT>(input, input, dv);


  pcl::gpu::DeviceArray<PointT> gpuInput;
  pcl::gpu::DeviceArray<PointT> gpuTarget;
  gpuInput.upload(input.points);
  gpuTarget.upload(target.points);

//  TOCK(a)
//  TOCK(a)
  float fitnessScore = 0.0f;
  cloud2GPU(gpuInput, gpuTarget, fitnessScore, input_index, target_index);

  return {fitnessScore};
}

oa::VectorDouble::size_type L2_PE_Problem::numOfObjectiveFunction()
{
  return 1;
}

oa::VectorDouble::size_type L2_PE_Problem::dimension()
{
  return 6;
}

std::pair<oa::VectorDouble, oa::VectorDouble> L2_PE_Problem::bounds()
{
  return bounds_;
}

void L2_PE_Problem::setInputCloud(const PointCloudT::Ptr& cloudSource)
{
  source_ = cloudSource;
}

void L2_PE_Problem::setTargetCloud(const PointCloudT::Ptr& cloudTarget)
{
  target_ = cloudTarget;

  float limit_[12];
  limit_[0] = -M_PI;
  limit_[1] = M_PI;
  limit_[2] = -M_PI;
  limit_[3] = M_PI;
  limit_[4] = -M_PI;
  limit_[5] = M_PI;
  //  limit_[3][0] = -1;
  //  limit_[3][1] = 1;
  //  limit_[4][0] = -1;
  //  limit_[4][1] = 1;
  //  limit_[5][0] = -1;
  //  limit_[5][1] = 1;
  Eigen::Vector4f centroid1;                   // 质心
  pcl::compute3DCentroid(*source_, centroid1); // 齐次坐标，（c0,c1,c2,1）

  Eigen::Vector4f centroid2;                   // 质心
  pcl::compute3DCentroid(*target_, centroid2); // 齐次坐标，（c0,c1,c2,1）

  auto l1 = std::pow(centroid1[0], 2) + std::pow(centroid1[1], 2) + std::pow(centroid1[2], 2);
  auto l2 = std::pow(centroid2[0], 2) + std::pow(centroid2[1], 2) + std::pow(centroid2[2], 2);
  auto limit = std::sqrt(l1) + std::sqrt(l2);
  limit_[6] = -limit;
  limit_[7] = limit;
  limit_[8] = -limit;
  limit_[9] = limit;
  limit_[10] = -limit;
  limit_[11] = limit;

  bounds_ = {{limit_[0], limit_[2], limit_[4], limit_[6], limit_[8], limit_[10]},
             {limit_[1], limit_[3], limit_[5], limit_[7], limit_[9], limit_[11]}};
}

oa::VectorDouble::value_type L2_PE_Problem::correspondenceEstimation()
{
  return 0.00001;
  //          return 1;
};