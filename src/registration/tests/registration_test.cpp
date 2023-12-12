//
// Created by XI on 2022/12/6.
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
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <PCR_Utility.h>

// OA Includes
#include "L2_PE_Problem.h"
#include "L2_SimpleProblem.h"
#include "MultipleClassTeachingLearningBasedOptimization.h"
#include "MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn.h"
#include "Problems.hpp"
#include "TeachingLearningBasedOptimization.h"
#include "TeachingLearningBasedWhaleOptimiziotn.h"

// dataset Includes
#include "../dataset/ReadPlyData.h"

// Registration Includes
#include "registration_visualization.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

TEST(tlboRegistraion, test1)
{
  auto begin = std::chrono::high_resolution_clock::now();
  L2_SimpleProblem probb;
  float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto source1 =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudSource;
  auto target1 =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudTarget;

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(source1);
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  PointCloudT::Ptr source(new PointCloudT);
  filter.filter(*source);

  filter.setInputCloud(target1);
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  PointCloudT::Ptr target(new PointCloudT);
  filter.filter(*target);

  std::cout << (*source).size() << std::endl;
  probb.setInputCloud(source);
  probb.setTargetCloud(target);

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  //  oa::multipleClassTeachingLearningBasedOptimization mctlbo(8, 5, 4);
  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = mctlbo.optimize(pop);

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  //    std::cout << elapsed.count() * 1e-9 << std::endl;
  printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
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

  pcl::transformPointCloud(*source, *trans, transform_2);
  std::cout << res.championFitnessScores()[0] << std::endl;
  std::cout << transform_2.matrix() << std::endl;

  showTrans(source, target, trans);

  ASSERT_TRUE(true);
}

TEST(mctlboRegistraion, test1)
{
  float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  PointCloudT::Ptr icp_trans(new PointCloudT);
  auto source =
      readPlyData::instance()->testPointCloud(readPlyData::LUCY, posAndRotation)->cloudSource;
  auto target =
      readPlyData::instance()->testPointCloud(readPlyData::LUCY, posAndRotation)->cloudTarget;
  L2_SimpleProblem fitProb;
  fitProb.setInputCloud(source);
  fitProb.setTargetCloud(target);
  //  PointCloudT::Ptr source(new PointCloudT);
  //  PointCloudT::Ptr target(new PointCloudT);
  //  pcl::RandomSample<pcl::PointXYZ> rs;
  //  rs.setInputCloud(source1);
  //  // 设置输出点的数量
  //  rs.setSample(10000);
  //
  //  // 下采样并输出到cloud_sample
  //  rs.filter(*source);
  //  calculateTransPointCloud<PointT>(*source1, *target, posAndRotation);
  //  pcl::VoxelGrid<pcl::PointXYZ> filter;
  //  filter.setInputCloud(source1);
  //  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  //  PointCloudT::Ptr source(new PointCloudT);
  //  filter.filter(*source);
  //
  //  filter.setInputCloud(target1);
  //  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  //  PointCloudT::Ptr target(new PointCloudT);
  //  filter.filter(*target);
  //
  std::cout << (*source).size() << std::endl;

  L2_PE_Problem probb;

  probb.setInputCloud(source);
  probb.setTargetCloud(target);

  oa::multipleClassTeachingLearningBasedOptimization mctlbo(24, 8, 4);
  //  oa::teachingLearningBasedOptimization mctlbo(20);
  float testTimeSum = 0.0f;
  float fitnessSum = 0.0f;
  //  for (size_t i = 0; i < 30; ++i) {
  oa::Problem prob{probb};
  oa::Population pop{prob, 30};
  auto begin = std::chrono::high_resolution_clock::now();
  auto res = mctlbo.optimize(pop);
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  //    testTimeSum += elapsed.count() * 1e-9;
  //    std::cout << i << std::endl;
  //    fitnessSum += fitProb.fitnessScore(res.championDecisionVariables())[0];
  //  }

  //  std::cout << testTimeSum / 30. << std::endl;
  //  std::cout << fitnessSum / 30. << std::endl;
  std::cout << elapsed.count() * 1e-9 << std::endl;
  //  printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
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

  pcl::transformPointCloud(*source, *trans, transform_2);
  //    std::cout << transform_2.matrix() << std::endl;
  //  std::cout << res.championFitnessScores()[0] << std::endl;

  //  PointCloudT::Ptr icp_trans(new PointCloudT);
  //  pcl::IterativeClosestPoint<PointT, PointT> icp1;
  //  icp1.setMaximumIterations(0);
  //  icp1.setInputSource(trans);
  //  icp1.setInputTarget(target);
  //  icp1.align(*icp_trans);
  //
  //  std::cout << icp1.getFitnessScore() << std::endl;

  showTrans(source, target, trans);

  ASSERT_TRUE(true);
}

TEST(mctlboRegistraion, test2)
{
  PointCloudT::Ptr cloudSource = std::make_shared<PointCloudT>();

  //  QFile inFile(R"(C:\Users\XI\Desktop\test1.csv)");
  //  QStringList lines; /*行数据*/
  //
  //  if (inFile.open(QIODevice::ReadOnly)) {
  //    QTextStream stream_text(&inFile);
  //    while (!stream_text.atEnd()) {
  //      lines.push_back(stream_text.readLine());
  //    }
  //    for (int j = 0; j < lines.size(); j++) {
  //      QString line = lines.at(j);
  //      QStringList split = line.split(","); /*列数据*/
  //      pcl::PointXYZ point;
  //      if (split.at(9).toFloat() > 130 || split.at(9).toFloat() < -130) {
  //        continue;
  //      }
  //      point.x = split.at(9).toFloat();
  //      point.y = split.at(10).toFloat();
  //      point.z = split.at(11).toFloat();
  //      cloudSource->emplace_back(point);
  //    }
  //    inFile.close();
  //  }

  L2_PE_Problem probb;
  PointCloudT::Ptr cloudTarget(new PointCloudT);
  pcl::io::loadPLYFile(R"(E:\CLionProjects\PointCloudRegistration\resource\navi\baseLine-5-31.ply)",
                       *cloudTarget);
  pcl::io::loadPLYFile(
      R"(E:\CLionProjects\PointCloudRegistration\resource\navi\2023-6-2\regis1.ply)", *cloudSource);

  probb.setInputCloud(cloudSource);
  probb.setTargetCloud(cloudTarget);

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(24, 6, 4);
  //  oa::teachingLearningBasedOptimization mctlbo(20);
  auto begin = std::chrono::high_resolution_clock::now();
  auto res = mctlbo.optimize(pop);
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  //    std::cout << elapsed.count() * 1e-9 << std::endl;
  printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
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

  pcl::transformPointCloud(*cloudSource, *trans, transform_2);
  std::cout << transform_2.matrix() << std::endl;
  std::cout << res.championFitnessScores()[0] << std::endl;

  PointCloudT::Ptr icp_trans(new PointCloudT);
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(100);
  icp.setInputSource(trans);
  icp.setInputTarget(cloudTarget);
  icp.align(*icp_trans);
  std::cout << icp.getFinalTransformation() << std::endl;
  std::cout << icp.getFitnessScore() << std::endl;

  showTrans(cloudSource, cloudTarget, icp_trans);

  ASSERT_TRUE(true);
}

TEST(tlbwoaRegistraion, test1)
{
  L2_SimpleProblem probb;

  float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto source =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudSource;
  auto target =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudTarget;

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  oa::TeachingLearningBasedWhaleOptimiziotn tlbwoa(20);
  //  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = tlbwoa.optimize(pop);

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

  pcl::transformPointCloud(*source, *trans, transform_2);
  std::cout << transform_2.matrix() << std::endl;

  showTrans(source, target, trans);

  ASSERT_TRUE(true);
}

TEST(mwgtlbwoRegistraion, test1)
{
  L2_SimpleProblem probb;

  float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
  auto source =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudSource;
  auto target =
      readPlyData::instance()->testPointCloud(readPlyData::BUNNY, posAndRotation)->cloudTarget;

  probb.setInputCloud(source);
  probb.setTargetCloud(target);

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  oa::MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn mwgtlbwo(8, 5, 4);
  //  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = mwgtlbwo.optimize(pop);

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

  pcl::transformPointCloud(*source, *trans, transform_2);
  std::cout << transform_2.matrix() << std::endl;

  showTrans(source, target, trans);

  ASSERT_TRUE(true);
}