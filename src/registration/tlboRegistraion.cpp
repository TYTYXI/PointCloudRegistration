//
// Created by XI on 2022/12/6.
//
#include "tlboRegistraion.h"

void tlboRegistraion::align()
{

  this->tlbo_.optimize();
  this->tlbo_.parameters(params_);

  std::cout << "aa" << std::endl;
  std::cout << this->tlbo_.teacher()->fitnessScore_ << std::endl;
  std::cout << this->tlbo_.teacher()->params_[0] << this->tlbo_.teacher()->params_[1]
            << this->tlbo_.teacher()->params_[2] << this->tlbo_.teacher()->params_[3]
            << this->tlbo_.teacher()->params_[4] << this->tlbo_.teacher()->params_[5] << std::endl;
}

Eigen::Matrix4f tlboRegistraion::finalMatrix() const
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << params_[3], params_[4], params_[5];
  transform.rotate(Eigen::AngleAxisf(params_[0], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(params_[1], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(params_[2], Eigen::Vector3f::UnitZ()));
  return transform.matrix();
}

tlboRegistraion::tlboRegistraion(const PointCloudT::Ptr& cloudSource,
                                 const PointCloudT::Ptr& cloudTarget)
    : tlbo_(ioa::teachingLearningBasedOptimization<L2_Simple<float, PointT>>(
          10, 20, L2_Simple<float, PointT>(cloudSource, cloudTarget)))
{
  tlbo_.targetFunc_.cloudSource_ = cloudSource;
  tlbo_.targetFunc_.cloudTarget_ = cloudTarget;
  tlbo_.targetFunc_.tree_.setInputCloud(tlbo_.targetFunc_.cloudTarget_);

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
  Eigen::Vector4f centroid1;                       // 质心
  pcl::compute3DCentroid(*cloudSource, centroid1); // 齐次坐标，（c0,c1,c2,1）

  Eigen::Vector4f centroid2;                       // 质心
  pcl::compute3DCentroid(*cloudTarget, centroid2); // 齐次坐标，（c0,c1,c2,1）

  auto l1 = std::pow(centroid1[0], 2) + std::pow(centroid1[1], 2) + std::pow(centroid1[2], 2);
  auto l2 = std::pow(centroid2[0], 2) + std::pow(centroid2[1], 2) + std::pow(centroid2[2], 2);
  auto limit = std::sqrt(l1) + std::sqrt(l2);
  limit_[6] = -limit;
  limit_[7] = limit;
  limit_[8] = -limit;
  limit_[9] = limit;
  limit_[10] = -limit;
  limit_[11] = limit;

  this->tlbo_.setLimits(limit_);
  this->tlbo_.initTLBO();
}
