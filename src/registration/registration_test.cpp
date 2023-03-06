//
// Created by XI on 2022/12/6.
//
// Qt
#include <QFile>

#include "gtest/gtest.h"

#include "../dataset/readPlyData.h"

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

#include "OptimizationAlgorithms/algorithm/MultipleClassTeachingLearningBasedOptimization.h"
#include "OptimizationAlgorithms/algorithm/MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn.h"
#include "OptimizationAlgorithms/algorithm/TeachingLearningBasedOptimization.h"
#include "OptimizationAlgorithms/algorithm/TeachingLearningBasedWhaleOptimiziotn.h"
#include "OptimizationAlgorithms/Problems.hpp"
// #include "tlboRegistraion.h"
#include "L2_SimpleProblem.h"

#include "registraion_visualization.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

TEST(tlboRegistraion, test1)
{
  L2_SimpleProblem probb;

  probb.setInputCloud(readPlyData::testPointCloud()->cloudSource);
  probb.setInputTarget(readPlyData::testPointCloud()->cloudTarget);

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  //  oa::multipleClassTeachingLearningBasedOptimization mctlbo(8, 5, 4);
  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = mctlbo.optimize(pop);

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

TEST(mctlboRegistraion, test1)
{
  L2_SimpleProblem probb;

  probb.setInputCloud(readPlyData::testPointCloud()->cloudSource);
  probb.setInputTarget(readPlyData::testPointCloud()->cloudTarget);

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(8, 5, 4);
  //  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = mctlbo.optimize(pop);

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

TEST(mctlboRegistraion, test2)
{
  PointCloudT::Ptr cloudSource = std::make_shared<PointCloudT>();

  QFile inFile(R"(C:\Users\XI\Desktop\test1.csv)");
  QStringList lines; /*行数据*/

  if (inFile.open(QIODevice::ReadOnly)) {
    QTextStream stream_text(&inFile);
    while (!stream_text.atEnd()) {
      lines.push_back(stream_text.readLine());
    }
    for (int j = 0; j < lines.size(); j++) {
      QString line = lines.at(j);
      QStringList split = line.split(","); /*列数据*/
      pcl::PointXYZ point;
      if (split.at(9).toFloat() > 130 || split.at(9).toFloat() < -130) {
        continue;
      }
      point.x = split.at(9).toFloat();
      point.y = split.at(10).toFloat();
      point.z = split.at(11).toFloat();
      cloudSource->emplace_back(point);
    }
    inFile.close();
  }

  L2_SimpleProblem probb;
  PointCloudT::Ptr cloudTarget(new PointCloudT);
  pcl::io::loadPLYFile(R"(C:\Users\XI\Desktop\baseLine_3.ply)", *cloudTarget);
  pcl::io::loadPLYFile(R"(C:\Users\XI\Desktop\baseLine_21.ply)", *cloudSource);

  probb.setInputCloud(cloudSource);
  probb.setInputTarget(cloudTarget);

  oa::Problem prob{probb};
  oa::Population pop{prob, 25};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(12, 5, 4);
  //  oa::teachingLearningBasedOptimization mctlbo(20);

  auto res = mctlbo.optimize(pop);

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

  showTrans(cloudSource, cloudTarget, trans);

  ASSERT_TRUE(true);
}

TEST(tlbwoaRegistraion, test1)
{
  L2_SimpleProblem probb;

  probb.setInputCloud(readPlyData::testPointCloud()->cloudSource);
  probb.setInputTarget(readPlyData::testPointCloud()->cloudTarget);

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

  pcl::transformPointCloud(*readPlyData::testPointCloud()->cloudSource, *trans, transform_2);
  std::cout << transform_2.matrix() << std::endl;

  showTrans(readPlyData::testPointCloud()->cloudSource, readPlyData::testPointCloud()->cloudTarget,
            trans);

  ASSERT_TRUE(true);
}

TEST(mwgtlbwoRegistraion, test1)
{
  L2_SimpleProblem probb;

  probb.setInputCloud(readPlyData::testPointCloud()->cloudSource);
  probb.setInputTarget(readPlyData::testPointCloud()->cloudTarget);

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

  pcl::transformPointCloud(*readPlyData::testPointCloud()->cloudSource, *trans, transform_2);
  std::cout << transform_2.matrix() << std::endl;

  showTrans(readPlyData::testPointCloud()->cloudSource, readPlyData::testPointCloud()->cloudTarget,
            trans);

  ASSERT_TRUE(true);
}