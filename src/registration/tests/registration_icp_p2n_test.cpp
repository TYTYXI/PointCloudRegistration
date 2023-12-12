//
// Created by XI on 2023/10/4.
//
// Qt
#include <QFile>
#include <QTextStream>

// STD Includes
#include <ctime>

// gtest Includes
#include "gtest/gtest.h"

// VTK Includes
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVertex.h>
#include <vtkVertexGlyphFilter.h>

// PCL Includes
#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h> //ndt头文件
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

// dataset Includes
#include "../dataset/ReadPlyData.h"

// Utilities Includes
#include "PCR_Utility.h"

// Registration Includes
#include "registration_visualization.h"

using PointNT = pcl::PointNormal;
using PointCloudNT = pcl::PointCloud<PointNT>;

TEST(icp_p2n_test, test1)
{
  float posAndRotation[6] = {-0.05, -0.02, 0.09, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto source =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudSource;
  auto target =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudTarget;

  PointCloudNT::Ptr source_with_normals(new PointCloudNT);
  PointCloudNT::Ptr target_with_normals(new PointCloudNT);
  pcl::copyPointCloud(*source, *source_with_normals);
  pcl::copyPointCloud(*target, *target_with_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  /*求取法线->对输入的点云图像中的一个点进行法向量求解*/
  // 法向量求解器
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> pointNormalEstimation;
  pointNormalEstimation.setNumberOfThreads(16);
  pointNormalEstimation.setRadiusSearch(0.03);
  pointNormalEstimation.setSearchMethod(tree);
  pointNormalEstimation.setInputCloud(source);
  pointNormalEstimation.compute(*source_with_normals);
  std::cout << source_with_normals->size() << std::endl;
  pointNormalEstimation.setNumberOfThreads(16);
  pointNormalEstimation.setRadiusSearch(0.03);
  pointNormalEstimation.setSearchMethod(tree);
  pointNormalEstimation.setInputCloud(target);
  pointNormalEstimation.compute(*target_with_normals);
  std::cout << target_with_normals->size() << std::endl;
  // KNN
  //  // 设置KdTree要求解的点云参数
  //  tree->setInputCloud(source);
  //  // 计算 source 法向量
  //  for (size_t l = 0; l < source->size(); ++l) {
  //    // 我们要求解的点,这个点的index你可以自己设置
  //    pcl::PointXYZ searchPoint = source->points[l];
  //    // 这是K近邻的半径
  //    float radius = 0.03;
  //    // 关键的数据indices
  //    std::vector<int> indices;
  //    // 每个点到searchPoint的距离(暂时用不到)
  //    std::vector<float> distance;
  //    tree->radiusSearch(searchPoint, radius, indices, distance);
  //
  //    // 输出参数1>平面数据(可以转化为法向量)
  //    Eigen::Vector4f planeParams;
  //    // 输出参数2>平面曲率
  //    float curvature;
  //    // 进行单个点的法向量求解
  //    pointNormalEstimation.computePointNormal(*source, indices, planeParams, curvature);
  //
  //    source_with_normals->push_back(pcl::PointNormal(
  //        searchPoint._PointXYZ::x, searchPoint._PointXYZ::y, searchPoint._PointXYZ::z,
  //        planeParams[0], planeParams[1], planeParams[2], curvature));
  //  }
  //  // 设置KdTree要求解的点云参数
  //  tree->setInputCloud(target);
  //  // 计算 target 法向量
  //  for (size_t l = 0; l < target->size(); ++l) {
  //    // 我们要求解的点,这个点的index你可以自己设置
  //    pcl::PointXYZ searchPoint = target->points[l];
  //    // 这是K近邻的半径
  //    float radius = 0.03;
  //    // 关键的数据indices
  //    std::vector<int> indices;
  //    // 每个点到searchPoint的距离(暂时用不到)
  //    std::vector<float> distance;
  //    tree->radiusSearch(searchPoint, radius, indices, distance);
  //
  //    // 输出参数1>平面数据(可以转化为法向量)
  //    Eigen::Vector4f planeParams;
  //    // 输出参数2>平面曲率
  //    float curvature;
  //    // 进行单个点的法向量求解
  //    pointNormalEstimation.computePointNormal(*target, indices, planeParams, curvature);
  //
  //    target_with_normals->push_back(pcl::PointNormal(
  //        searchPoint._PointXYZ::x, searchPoint._PointXYZ::y, searchPoint._PointXYZ::z,
  //        planeParams[0], planeParams[1], planeParams[2], curvature));
  //  }

  PointCloudNT::Ptr icp_trans_with_normals(new PointCloudNT);

  auto begin = std::chrono::high_resolution_clock::now();
  pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp1;
//  icp1.setEuclideanFitnessEpsilon(0.0001);
  icp1.setMaximumIterations(100);
  icp1.setInputSource(source_with_normals);
  icp1.setInputTarget(target_with_normals);
  icp1.align(*icp_trans_with_normals);
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  //    std::cout << elapsed.count() * 1e-9 << std::endl;
  printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
  std::cout << "final-icp " << icp1.getFitnessScore() << std::endl;
  PointCloudT::Ptr icp_trans(new PointCloudT);
  pcl::copyPointCloud(*icp_trans_with_normals, *icp_trans);
  showTrans(source, target, icp_trans);
  ASSERT_TRUE(true);
}
