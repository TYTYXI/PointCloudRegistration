//
// Created by XI on 2022/12/6.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_TLBOREGISTRAION_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_TLBOREGISTRAION_H_

#include "registration_global.h"

#include "dist.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "../algorithm/TeachingLearningBasedOptimization.h"

class REGISTRATION_EXPORT tlboRegistraion
{

  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

public:
public:
  tlboRegistraion(const PointCloudT::Ptr& cloudSource, const PointCloudT::Ptr& cloudTarget);

  void align();

  Eigen::Matrix4f finalMatrix() const;

private:
  Eigen::Matrix4f finalTrans_;
  float params_[6];
  ioa::teachingLearningBasedOptimization<L2_Simple<float, PointT>> tlbo_;
};
#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_TLBOREGISTRAION_H_
