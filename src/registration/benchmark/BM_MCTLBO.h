//
// Created by XI on 2023/10/12.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_MCTLBO_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_MCTLBO_H_

// STD Includes
#include <chrono>

// Qt Includes
#include <QFile>

// Google BenchMark Includes
#include "benchmark/benchmark.h"

// PCL Includes
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <PCR_Utility.h>
#include <registration_visualization.h>

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

void BM_EFFICIENCY_MCTLBO(benchmark::State& state)
{
  QFile saveBenchMarkTest("BM_EFFICIENCY_MCTLBO.csv");
  if (saveBenchMarkTest.open(QIODevice::Append | QIODevice::Text)) {
    for (auto _ : state) {
      float posAndRotation[6] = {-100.5, -100.2, 100.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
      auto source1 = readPlyData::instance()
                         ->testPointCloud(readPlyData::XYZRGB_DRAGON, posAndRotation)
                         ->cloudSource;
      auto target1 = readPlyData::instance()
                         ->testPointCloud(readPlyData::XYZRGB_DRAGON, posAndRotation)
                         ->cloudTarget;

      PointCloudT::Ptr source(new PointCloudT);
      PointCloudT::Ptr target(new PointCloudT);

      pcl::RandomSample<pcl::PointXYZ> rs;

      rs.setInputCloud(source1);
      // 设置输出点的数量
      int y = state.range(0) / 10;
      if (y != 0) {
        if (state.range(0) - 10 * y == 0) {
          y = y - 1;
        }
      }
      int x = state.range(0) % 10 == 0 ? 10 : state.range(0) % 10;
      rs.setSample(int((float)x / 10. * 2000. * pow(10, y)));
      // 下采样并输出到cloud_sample
      rs.filter(*source);
      calculateTransPointCloud<PointT>(*source, *target, posAndRotation);
      // 下采样并输出到cloud_sample
      std::cout << "source size = " << (*source).size() << std::endl;

      L2_PE_Problem probb;
      probb.setInputCloud(source);
      probb.setTargetCloud(target);

      L2_SimpleProblem fitProb;
      fitProb.setInputCloud(source);
      fitProb.setTargetCloud(target);

      //    for (size_t i = 0; i < 30; ++i) {
      oa::Problem prob{probb};
      oa::Population pop{
          prob,30
      };
      oa::multipleClassTeachingLearningBasedOptimization mctlbo(12, 6, 4);
      //  oa::teachingLearningBasedOptimization mctlbo(20);
      PointCloudT::Ptr trans(new PointCloudT);
      auto begin = std::chrono::high_resolution_clock::now();
      auto res = mctlbo.optimize(pop);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      std::cout << elapsed.count() * 1e-9 << std::endl;
      //      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      //    }

      QString out;
      out.append(QString::number((*source).size()) + "," +
                 QString::number(fitProb.fitnessScore(res.championDecisionVariables())[0]) + "," +
                 QString::number(elapsed.count() * 1e-9) + "\n");
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
      //            showTrans(source, target, trans);
    }
  }
}

void BM_ACCURACY_MCTLBO(benchmark::State& state)
{
  for (auto _ : state) {
    QFile saveBenchMarkTest("BM_ACCURACY_MCTLBO.csv");
    if (saveBenchMarkTest.open(QIODevice::Append | QIODevice::Text)) {
      float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
      auto source = readPlyData::instance()
                        ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                        ->cloudSource;
      auto target = readPlyData::instance()
                        ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                        ->cloudTarget;

      std::cout << source->size() << std::endl;

      L2_PE_Problem probb;
      probb.setInputCloud(source);
      probb.setTargetCloud(target);

      L2_SimpleProblem fitProb;
      fitProb.setInputCloud(source);
      fitProb.setTargetCloud(target);

      //    for (size_t i = 0; i < 30; ++i) {
      oa::Problem prob{probb};
      oa::Population pop{prob, 25};
      oa::multipleClassTeachingLearningBasedOptimization mctlbo(12, 6, 4);
      //  oa::teachingLearningBasedOptimization mctlbo(20);
      PointCloudT::Ptr trans(new PointCloudT);
      auto begin = std::chrono::high_resolution_clock::now();
      auto res = mctlbo.optimize(pop);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      std::cout << elapsed.count() * 1e-9 << std::endl;
      //      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      //    }

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

      //      std::cout << "icp start" << std::endl;
      pcl::IterativeClosestPoint<PointT, PointT> icp1;
      icp1.setMaximumIterations(10);
      //      icp1.setInputSource(source);
      //      icp1.setInputTarget(target);
      //      icp1.align(*trans, transform_2.matrix());

      //      std::cout << "icp end" << std::endl;
      QString out;
      auto out_std = fitProb.fitnessScore(res.championDecisionVariables())[0];
      out.append(QString::number(out_std) + "," + QString::number(elapsed.count() * 1e-9) + "\n");
      std::cout << "tlbo " << out_std << std::endl;
      //      std::cout << "icp " << icp1.getFitnessScore() << std::endl;
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
      //      showTrans(source, target, trans);
    }
  }
}

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_MCTLBO_H_
