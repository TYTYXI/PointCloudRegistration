//
// Created by XI on 2023/9/30.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_PE_PROBLEM_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_PE_PROBLEM_H_

#include <pcl/common/transforms.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>


#include "Types.h"

#include "registration_global.h"

class REGISTRATION_EXPORT L2_PE_Problem
{
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

public:
  L2_PE_Problem();
  virtual ~L2_PE_Problem();
  oa::VectorDouble fitnessScore(const oa::VectorDouble& dv);
  oa::VectorDouble::size_type numOfObjectiveFunction();
  oa::VectorDouble::size_type dimension();
  oa::VectorDouble::value_type correspondenceEstimation();
  std::pair<oa::VectorDouble, oa::VectorDouble> bounds();

  void setInputCloud(const PointCloudT::Ptr& cloudSource);
  void setTargetCloud(const PointCloudT::Ptr& cloudTarget);

private:
  PointCloudT::Ptr source_;
  PointCloudT::Ptr target_;
  pcl::gpu::DeviceArray<PointT> gpuTarget_;
  std::pair<oa::VectorDouble, oa::VectorDouble> bounds_;
};

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_PE_PROBLEM_H_
