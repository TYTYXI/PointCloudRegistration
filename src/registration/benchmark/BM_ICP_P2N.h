//
// Created by XI on 2023/10/12.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_ICP_P2N_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_ICP_P2N_H_

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

using PointNT = pcl::PointNormal;
using PointCloudNT = pcl::PointCloud<PointNT>;

void BM_EFFICIENCY_ICP_P2N(benchmark::State& state)
{
  for (auto _ : state) {
    float posAndRotation[6] = {-100.5, -100.2, 100.9, 2. / 3. * M_PI, -M_PI / 2, M_PI / 3};
    PointCloudT::Ptr source(new PointCloudT);
    PointCloudT::Ptr target(new PointCloudT);
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

    PointCloudNT::Ptr source_with_normals(new PointCloudNT);
    PointCloudNT::Ptr target_with_normals(new PointCloudNT);
    pcl::copyPointCloud(*source, *source_with_normals);
    pcl::copyPointCloud(*target, *target_with_normals);

    auto begin = std::chrono::high_resolution_clock::now();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    /*求取法线->对输入的点云图像中的一个点进行法向量求解*/
    // 法向量求解器
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> pointNormalEstimation;
    pointNormalEstimation.setNumberOfThreads(16);
    pointNormalEstimation.setRadiusSearch(0.03);
    pointNormalEstimation.setSearchMethod(tree1);
    pointNormalEstimation.setInputCloud(source);
    pointNormalEstimation.compute(*source_with_normals);
    std::cout << source_with_normals->size() << std::endl;
    pointNormalEstimation.setNumberOfThreads(16);
    pointNormalEstimation.setRadiusSearch(0.03);
    pointNormalEstimation.setSearchMethod(tree2);
    pointNormalEstimation.setInputCloud(target);
    pointNormalEstimation.compute(*target_with_normals);
    std::cout << target_with_normals->size() << std::endl;
    PointCloudNT::Ptr icp_trans_with_normals(new PointCloudNT);

    pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp1;
    icp1.setEuclideanFitnessEpsilon(0.0001);
    icp1.setMaximumIterations(100);
    icp1.setInputSource(source_with_normals);
    icp1.setInputTarget(target_with_normals);
    icp1.align(*icp_trans_with_normals);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    std::cout << " score: " << icp1.getFitnessScore() << std::endl;
    printf("Time measured: %.9f seconds.\n", elapsed.count() * 1e-9);
  }
}

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_BENCHMARK_BM_ICP_P2N_H_
