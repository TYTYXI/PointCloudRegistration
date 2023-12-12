//
// Created by XI on 2023/9/30.
//

#ifndef POINTCLOUDREGISTRATION_SRC_UTILITIES_PCR_UTILITY_H_
#define POINTCLOUDREGISTRATION_SRC_UTILITIES_PCR_UTILITY_H_

// PCL Includes
#include <pcl/common/transforms.h>

// OA Includes
#include "Types.h"

#include "UtilitiesHlobal.h"

#define TICK(x) auto begin##x = std::chrono::high_resolution_clock::now();

#define TOCK(x)                                                                                    \
  auto end##x = std::chrono::high_resolution_clock::now();                                         \
  auto elapsed##x = std::chrono::duration_cast<std::chrono::nanoseconds>(end##x - begin##x);       \
  printf(#x " Time measured: %.9f seconds.\n", elapsed##x.count() * 1e-9);

class UTILITIES_EXPORT aaa
{};

template <typename T>
UTILITIES_EXPORT inline void calculateTransPointCloud(const pcl::PointCloud<T>& cloudIn,
                                                      pcl::PointCloud<T>& cloudOut,
                                                      const oa::VectorDouble& dv)
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << dv[3], dv[4], dv[5];
  transform.rotate(Eigen::AngleAxisf(dv[0], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(dv[1], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(dv[2], Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(cloudIn, cloudOut, transform);
}

template <typename T>
UTILITIES_EXPORT inline void calculateTransPointCloud(const pcl::PointCloud<T>& cloudIn,
                                                      pcl::PointCloud<T>& cloudOut, float* dv)
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << dv[3], dv[4], dv[5];
  transform.rotate(Eigen::AngleAxisf(dv[0], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(dv[1], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(dv[2], Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(cloudIn, cloudOut, transform);
}

#endif // POINTCLOUDREGISTRATION_SRC_UTILITIES_PCR_UTILITY_H_
