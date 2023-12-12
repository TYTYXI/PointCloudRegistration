//
// Created by XI on 2023/10/12.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_IA_RANSAC_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_IA_RANSAC_H_

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

void BM_EFFICIENCY_IA_RANSAC(benchmark::State& state)
{
  QFile saveBenchMarkTest("BM_EFFICIENCY_IA_RANSAC.csv");
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
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
      ne_src.setInputCloud(source);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
      ne_src.setSearchMethod(tree_src);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud<pcl::Normal>);
      ne_src.setRadiusSearch(0.2);
      ne_src.compute(*cloud_src_normals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
      ne_tgt.setInputCloud(target);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>());
      ne_tgt.setSearchMethod(tree_tgt);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud<pcl::Normal>);
      // ne_tgt.setKSearch(20);
      ne_tgt.setRadiusSearch(0.2);
      ne_tgt.compute(*cloud_tgt_normals);

      // 计算FPFH
      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
      fpfh_src.setInputCloud(source);
      fpfh_src.setInputNormals(cloud_src_normals);
      pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
      fpfh_src.setSearchMethod(tree_src_fpfh);
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(
          new pcl::PointCloud<pcl::FPFHSignature33>());
      fpfh_src.setRadiusSearch(0.5);
      fpfh_src.compute(*fpfhs_src);
      std::cout << "compute *cloud_src fpfh" << endl;

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
      fpfh_tgt.setInputCloud(target);
      fpfh_tgt.setInputNormals(cloud_tgt_normals);
      pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
      fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(
          new pcl::PointCloud<pcl::FPFHSignature33>());
      fpfh_tgt.setRadiusSearch(0.5);
      fpfh_tgt.compute(*fpfhs_tgt);
      std::cout << "compute *cloud_tgt fpfh" << endl;

      // SAC配准
      pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
      scia.setInputSource(source);
      scia.setInputTarget(target);
      scia.setSourceFeatures(fpfhs_src);
      scia.setTargetFeatures(fpfhs_tgt);
      PointCloudT::Ptr sac_result(new PointCloudT);
      scia.align(*sac_result);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      std::cout << " score: " << scia.getFitnessScore() << std::endl;
      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      QString out;
      out.append(QString::number((*source).size()) + "," + QString::number(scia.getFitnessScore()) +
                 "," + QString::number(elapsed.count() * 1e-9) + "\n");
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
      //      showTrans(source, target, sac_result);
    }
  }
}

void BM_ACCURACY_IA_RANSAC(benchmark::State& state)
{
  for (auto _ : state) {
    QFile saveBenchMarkTest("BM_ACCURACY_IA_RANSAC.csv");
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
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
      ne_src.setInputCloud(source);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
      ne_src.setSearchMethod(tree_src);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud<pcl::Normal>);
      ne_src.setRadiusSearch(0.2);
      ne_src.compute(*cloud_src_normals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
      ne_tgt.setInputCloud(target);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>());
      ne_tgt.setSearchMethod(tree_tgt);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud<pcl::Normal>);
      // ne_tgt.setKSearch(20);
      ne_tgt.setRadiusSearch(0.2);
      ne_tgt.compute(*cloud_tgt_normals);

      // 计算FPFH
      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
      fpfh_src.setInputCloud(source);
      fpfh_src.setInputNormals(cloud_src_normals);
      pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
      fpfh_src.setSearchMethod(tree_src_fpfh);
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(
          new pcl::PointCloud<pcl::FPFHSignature33>());
      fpfh_src.setRadiusSearch(0.5);
      fpfh_src.compute(*fpfhs_src);
      std::cout << "compute *cloud_src fpfh" << endl;

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
      fpfh_tgt.setInputCloud(target);
      fpfh_tgt.setInputNormals(cloud_tgt_normals);
      pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
      fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(
          new pcl::PointCloud<pcl::FPFHSignature33>());
      fpfh_tgt.setRadiusSearch(0.5);
      fpfh_tgt.compute(*fpfhs_tgt);
      std::cout << "compute *cloud_tgt fpfh" << endl;

      // SAC配准
      pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
      scia.setInputSource(source);
      scia.setInputTarget(target);
      scia.setSourceFeatures(fpfhs_src);
      scia.setTargetFeatures(fpfhs_tgt);
      PointCloudT::Ptr sac_result(new PointCloudT);
      scia.align(*sac_result);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      //    std::cout << elapsed.count() * 1e-9 << std::endl;
      printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
      //  oa::teachingLearningBasedOptimization mctlbo(20);
      QString out;
      out.append(QString::number(scia.getFitnessScore()) + "," +
                 QString::number(elapsed.count() * 1e-9) + "\n");
      saveBenchMarkTest.write(out.toStdString().c_str());
      saveBenchMarkTest.close();
    }
  }
}

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_IA_RANSAC_H_
