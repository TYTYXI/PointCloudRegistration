//
// Created by XI on 2023/10/12.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_ICP_P2P_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_ICP_P2P_H_

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

void BM_EFFICIENCY_ICP_P2P(benchmark::State& state)
{
  QFile saveBenchMarkTest("BM_ICP_P2P.csv");
  if (saveBenchMarkTest.open(QIODevice::Append | QIODevice::Text)) {

    for (auto _ : state) {
      float posAndRotation[6] = {-100.5, -100.2, 100.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
      PointCloudT::Ptr source(new PointCloudT);
      PointCloudT::Ptr target(new PointCloudT);
      PointCloudT::Ptr icp_trans(new PointCloudT);
      auto source1 = readPlyData::instance()
                         ->testPointCloud(readPlyData::XYZRGB_DRAGON, posAndRotation)
                         ->cloudSource;
      auto target1 = readPlyData::instance()
                         ->testPointCloud(readPlyData::XYZRGB_DRAGON, posAndRotation)
                         ->cloudTarget;
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

      auto begin = std::chrono::high_resolution_clock::now();
      pcl::IterativeClosestPoint<PointT, PointT> icp1;
      icp1.setEuclideanFitnessEpsilon(0.00001);
      icp1.setMaximumIterations(100);
      icp1.setInputSource(source);
      icp1.setInputTarget(target);
      icp1.align(*icp_trans);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      //    std::cout << elapsed.count() * 1e-9 << std::endl;
      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      //  oa::teachingLearningBasedOptimization mctlbo(20);
      QString out;
      out.append(QString::number((*source).size()) + "," + QString::number(icp1.getFitnessScore()) +
                 "," + QString::number(elapsed.count() * 1e-9) + "\n");
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
    }
  }
}

void BM_ACCURACY_ICP_P2P(benchmark::State& state)
{
  for (auto _ : state) {
    QFile saveBenchMarkTest("BM_ACCURACY_ICP_P2P.csv");
    if (saveBenchMarkTest.open(QIODevice::Append | QIODevice::Text)) {

      float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
      PointCloudT::Ptr icp_trans(new PointCloudT);
      auto source = readPlyData::instance()
                        ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                        ->cloudSource;
      auto target = readPlyData::instance()
                        ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                        ->cloudTarget;
      std::cout << "source size = " << (*source).size() << std::endl;

      auto begin = std::chrono::high_resolution_clock::now();
      pcl::IterativeClosestPoint<PointT, PointT> icp1;
      icp1.setEuclideanFitnessEpsilon(0.00001);
      icp1.setMaximumIterations(100);
      icp1.setInputSource(source);
      icp1.setInputTarget(target);
      icp1.align(*icp_trans);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      //    std::cout << elapsed.count() * 1e-9 << std::endl;
      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      //  oa::teachingLearningBasedOptimization mctlbo(20);
      QString out;
      out.append(QString::number(icp1.getFitnessScore()) + "," +
                 QString::number(elapsed.count() * 1e-9) + "\n");
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
    }
  }
}

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_ICP_P2P_H_
