//
// Created by XI on 2023/2/24.
//

#include "gtest/gtest.h"

#include "../dataset/ReadPlyData.h"

#include <ctime>
#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QTextStream>

#include "OptimizationAlgorithms/algorithm/WhaleOptimization.h"
#include "OptimizationAlgorithms/Problems.hpp"
// #include "tlboRegistraion.h"
#include "L2_SimpleProblem.h"

#include "registration_visualization.h"

 using PointT=pcl::PointXYZ ;
using PointCloudT= pcl::PointCloud<PointT> ;

TEST(WOARegistraion, test1)
{
  L2_SimpleProblem probb;

  probb.setInputCloud(readPlyData::testPointCloud()->cloudSource);
  probb.setInputTarget(readPlyData::testPointCloud()->cloudTarget);

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  oa::WhaleOptimization woa(30);
  //  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = woa.optimize(pop);

  PointCloudT::Ptr trans(new PointCloudT);

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << res.championDecisionVariables()[3],
      res.championDecisionVariables()[4], res.championDecisionVariables()[5];
  transform_2.rotate(
      Eigen::AngleAxisf(res.championDecisionVariables()[0], Eigen::Vector3f::UnitX()));
  transform_2.rotate(
      Eigen::AngleAxisf(res.championDecisionVariables()[1], Eigen::Vector3f::UnitY()));
  transform_2.rotate(
      Eigen::AngleAxisf(res.championDecisionVariables()[2], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*readPlyData::testPointCloud()->cloudSource, *trans, transform_2);
  std::cout << transform_2.matrix() << std::endl;

  showTrans(readPlyData::testPointCloud()->cloudSource, readPlyData::testPointCloud()->cloudTarget,
            trans);

  ASSERT_TRUE(true);
}