//
// Created by XI on 2022/12/6.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_TLBOREGISTRAION_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_TLBOREGISTRAION_H_

#include "registration_global.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "../algorithm/TeachingLearningBasedOptimization.h"

class REGISTRATION_EXPORT tlboRegistraion
{

  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

public:
  template <class T>
  struct TargetFunc
  {
    typedef T ParamsType;
    typedef T ResultType;

  public:
    TargetFunc(PointCloudT::Ptr cloudSource, PointCloudT::Ptr cloudTarget)
        : cloudSource_(cloudSource)
        , cloudTarget_(cloudTarget)
    {
      tree_.setInputCloud(cloudTarget_);
    }

    pcl::KdTreeFLANN<PointT> tree_;
    PointCloudT::ConstPtr cloudSource_;
    PointCloudT::ConstPtr cloudTarget_;

    static constexpr int const numOfParams = 6;

    ResultType operator()(ResultType* a) const
    {
      auto fitness_score = 0.0;
      //  For each point in the source dataset
      int nr = 0;
      std::vector<int> k_indice(1);
      std::vector<float> distance(1);
      int K = 2;
      std::vector<int> ptIdxByKNN(K); // 存储查询点近邻索引
      std::vector<float> ptKNN(K);    // 存储近邻点对应距离平方

      Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
      transform_2.translation() << a[3], a[4], a[5];
      transform_2.rotate(Eigen::AngleAxisf(a[0], Eigen::Vector3f::UnitX()));
      transform_2.rotate(Eigen::AngleAxisf(a[1], Eigen::Vector3f::UnitY()));
      transform_2.rotate(Eigen::AngleAxisf(a[2], Eigen::Vector3f::UnitZ()));

      pcl::PointCloud<PointT> cloud_trans;
      pcl::transformPointCloud(*cloudSource_, cloud_trans, transform_2);

      // warning 记得delete
      float* qurey = new float[3];
      for (const auto& point : cloud_trans) {
        // Find its nearest neighbor in the target
        tree_.nearestKSearch(point, K, ptIdxByKNN, ptKNN); // 执行K近邻搜索

        // Deal with occlusions (incomplete targets)
        if (distance[0] <= std::numeric_limits<float>::max()) {
          // Add to the fitness score
          fitness_score += distance[0];
          nr++;
        }
      }
      delete[] qurey;

      if (nr > 0)
        return (fitness_score / (float)nr);
      return (std::numeric_limits<float>::max());
    }
  };

  tlboRegistraion(PointCloudT::Ptr cloudSource, PointCloudT::Ptr cloudTarget)
      : tlbo_(ioa::teachingLearningBasedOptimization<TargetFunc<float>>(
            20, 20, TargetFunc<float>(cloudSource, cloudTarget)))
  {
    tlbo_.targetFunc_.cloudSource_ = cloudSource;
    tlbo_.targetFunc_.cloudTarget_ = cloudTarget;
    tlbo_.targetFunc_.tree_.setInputCloud(tlbo_.targetFunc_.cloudSource_);
  }

  void align();
private:
  Eigen::Matrix4f finalTrans_;
  ioa::teachingLearningBasedOptimization<TargetFunc<float>> tlbo_;
};
#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_TLBOREGISTRAION_H_
