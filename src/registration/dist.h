//
// Created by XI on 2022/12/7.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_DIST_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_DIST_H_

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

template <class T, typename PointT>
struct L2_Simple
{
  typedef T ParamsType;
  typedef T ResultType;

  L2_Simple(typename pcl::PointCloud<PointT>::Ptr cloudSource,
             typename pcl::PointCloud<PointT>::Ptr cloudTarget)
      : cloudSource_(cloudSource)
      , cloudTarget_(cloudTarget)
  {
    tree_.setInputCloud(cloudTarget_);
  }

  pcl::KdTreeFLANN<PointT> tree_;
  typename pcl::PointCloud<PointT>::ConstPtr cloudSource_;
  typename pcl::PointCloud<PointT>::ConstPtr cloudTarget_;

  static constexpr int const numOfParams = 6;

  ResultType operator()(ResultType* a) const
  {
    auto fitnessScore = 0.0;
    //  For each point in the source dataset
    int nr = 0;
    std::vector<int> k_indice(1);
    std::vector<float> distance(1);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << a[3], a[4], a[5];
    transform.rotate(Eigen::AngleAxisf(a[0], Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(a[1], Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(a[2], Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<PointT> cloudTrans;
    pcl::transformPointCloud(*cloudSource_, cloudTrans, transform);

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
      return (fitnessScore / (float)nr);
    return (std::numeric_limits<float>::max());
  }
};

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_DIST_H_
