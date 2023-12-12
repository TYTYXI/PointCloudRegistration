//
// Created by XI on 2023/10/12.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_IA_FPCS_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_IA_FPCS_H_

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

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

void BM_EFFICIENCY_IA_FPCS(benchmark::State& state)
{
  QFile saveBenchMarkTest("BM_IA_FPCS.csv");
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
      // 四点法配准
      PointCloudT::Ptr pcs(new PointCloudT);
      pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;

      fpcs.setInputSource(source); // 输入待配准点云
      fpcs.setInputTarget(target); // 输入目标点云

      // 参数设置
      fpcs.setApproxOverlap(0.7); // 两点云重叠度
      fpcs.setDelta(0.5);         // Bunny
      fpcs.setMaxComputationTime(50);
      // fpcs.setDelta(0.5);//hippo
      if ((*source).size() < 3000) {
        std::cout << int((float)(*source).size()) / 8 << std::endl;
        fpcs.setNumberOfSamples(int((float)(*source).size()) / 5);
      } else {
        fpcs.setNumberOfSamples(1000);
      }
      fpcs.align(*pcs);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      std::cout << " score: " << fpcs.getFitnessScore() << std::endl;
      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      QString out;
      out.append(QString::number((*source).size()) + "," + QString::number(fpcs.getFitnessScore()) +
                 "," + QString::number(elapsed.count() * 1e-9) + "\n");
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
    }
  }
}

void BM_ACCURACY_IA_FPCS(benchmark::State& state)
{
  for (auto _ : state) {
    QFile saveBenchMarkTest("BM_ACCURACY_IA_FPCS.csv");
    if (saveBenchMarkTest.open(QIODevice::Append | QIODevice::Text)) {
      float posAndRotation[6] = {-0.5, -0.2, 0.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
      auto source = readPlyData::instance()
                        ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                        ->cloudSource;
      auto target = readPlyData::instance()
                        ->testPointCloud(readPlyData::ARMADILLO, posAndRotation)
                        ->cloudTarget;
      std::cout << "source size = " << (*source).size() << std::endl;

      auto begin = std::chrono::high_resolution_clock::now();
      // 四点法配准
      PointCloudT::Ptr pcs(new PointCloudT);
      pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;

      fpcs.setInputSource(source); // 输入待配准点云
      fpcs.setInputTarget(target); // 输入目标点云

      // 参数设置
      fpcs.setApproxOverlap(0.7); // 两点云重叠度
      fpcs.setDelta(0.5);         // Bunny
      fpcs.setMaxComputationTime(50);
      // fpcs.setDelta(0.5);//hippo
      if ((*source).size() < 3000) {
        std::cout << int((float)(*source).size()) / 8 << std::endl;
        fpcs.setNumberOfSamples(int((float)(*source).size()) / 5);
      } else {
        fpcs.setNumberOfSamples(1000);
      }
      fpcs.align(*pcs);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      std::cout << " score: " << fpcs.getFitnessScore() << std::endl;
      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      QString out;
      out.append(QString::number(fpcs.getFitnessScore()) + "," +
                 QString::number(elapsed.count() * 1e-9) + "\n");
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
    }
  }
}

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_IA_FPCS_H_
