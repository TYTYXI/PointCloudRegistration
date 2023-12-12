//
// Created by XI on 2023/1/4.
//

#include "L2_SimpleProblem.h"

L2_SimpleProblem::L2_SimpleProblem()
{
}

L2_SimpleProblem::~L2_SimpleProblem()
{
}

oa::VectorDouble L2_SimpleProblem::fitnessScore(const oa::VectorDouble& dv)
{
  auto fitnessScore = 0.0;
  //  For each point in the source dataset
  int nr = 0;
  std::vector<int> k_indice(1);
  std::vector<float> distance(1);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << dv[3], dv[4], dv[5];
  transform.rotate(Eigen::AngleAxisf(dv[0], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(dv[1], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(dv[2], Eigen::Vector3f::UnitZ()));

  pcl::PointCloud<PointT> cloudTrans;
  pcl::transformPointCloud(*source_, cloudTrans, transform);

  // warning 记得delete
  float* qurey = new float[3];
  for (const auto& point : cloudTrans) {
    // Find its nearest neighbor in the target
    tree_.nearestKSearch(point, 1, k_indice, distance); // 执行K近邻搜索

    // Deal with occlusions (incomplete targets)
    if (distance[0] <= std::numeric_limits<float>::max()) {
      // Add to the fitness score
      fitnessScore += distance[0];
      nr++;
    }
  }
  delete[] qurey;

  if (nr > 0)
    return {(fitnessScore / (float)nr)};
  return {(std::numeric_limits<float>::max())};
}

oa::VectorDouble::size_type L2_SimpleProblem::numOfObjectiveFunction()
{
  return 1;
}

oa::VectorDouble::size_type L2_SimpleProblem::dimension()
{
  return 6;
}

std::pair<oa::VectorDouble, oa::VectorDouble> L2_SimpleProblem::bounds()
{
  return bounds_;
}

void L2_SimpleProblem::setInputCloud(const PointCloudT::Ptr& cloudSource)
{
  source_ = cloudSource;
}

void L2_SimpleProblem::setTargetCloud(const PointCloudT::Ptr& cloudTarget)
{
  target_ = cloudTarget;
  tree_.setInputCloud(cloudTarget);

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

oa::VectorDouble::value_type L2_SimpleProblem::correspondenceEstimation()
{
  return 0.00001;
  //          return 1;
};