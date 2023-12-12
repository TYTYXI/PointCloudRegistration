//
// Created by XI on 2023/10/7.
//
// STD Includes
#include <ctime>

#include <omp.h>

// PCL Includes
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "gtest/gtest.h"

using pcl::NormalEstimation;
using pcl::search::KdTree;
using PointT = pcl::PointXYZ;
typedef pcl::PointCloud<PointT> PointCloud;

// dataset Includes
#include "ReadPlyData.h"

// Registration Includes
#include "registration_visualization.h"

TEST(ia_fpcs, test1)
{
      float posAndRotation[6] = {-100.5, -100.2, 100.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto source = readPlyData::instance()
                    ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                    ->cloudSource;
  auto target = readPlyData::instance()
                    ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                    ->cloudTarget;

  auto begin = std::chrono::high_resolution_clock::now();
  std::cout << source->size() << std::endl;
  // 四点法配准
  PointCloud::Ptr pcs(new PointCloud);
  pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;

  fpcs.setInputSource(source); // 输入待配准点云
  fpcs.setInputTarget(target); // 输入目标点云

  // 参数设置
  fpcs.setApproxOverlap(0.7); // 两点云重叠度
  fpcs.setDelta(0.5);         // Bunny
  // fpcs.setDelta(0.5);//hippo
  fpcs.setMaxComputationTime(50);
  fpcs.setNumberOfSamples(1000);
  Eigen::Matrix4f tras;
  fpcs.align(*pcs);
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  //    std::cout << elapsed.count() * 1e-9 << std::endl;
  printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
  std::cout << "final-fpcs " << fpcs.getFitnessScore() << std::endl;
  showTrans(source, target, pcs);
  ASSERT_TRUE(true);
}