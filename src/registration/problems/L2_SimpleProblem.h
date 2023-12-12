//
// Created by XI on 2023/1/4.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_SIMPLEPROBLEM_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_SIMPLEPROBLEM_H_

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <utility>

#include "Types.h"

#include "registration_global.h"

class REGISTRATION_EXPORT L2_SimpleProblem
{
   using PointT=pcl::PointXYZ ;
  using PointCloudT= pcl::PointCloud<PointT> ;

public:
  L2_SimpleProblem();
  virtual ~L2_SimpleProblem();
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
  std::pair<oa::VectorDouble, oa::VectorDouble> bounds_;
  pcl::KdTreeFLANN<PointT> tree_;
};

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_L2_SIMPLEPROBLEM_H_
