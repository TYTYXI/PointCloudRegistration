//
// Created by XI on 2022/12/6.
//

#include "benchmark/benchmark.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../../dataset/readPlyData.h"
#include "L2_SimpleProblem.h"
#include "OptimizationAlgorithms/algorithm/MultipleClassTeachingLearningBasedOptimization.h"
#include "OptimizationAlgorithms/algorithm/TeachingLearningBasedOptimization.h"
#include "OptimizationAlgorithms/Problems.hpp"

BENCHMARK_MAIN();

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void BM_MCTLBO(benchmark::State& state)
{
  for (auto _ : state) {
    L2_SimpleProblem probb;

    probb.setInputCloud(readPlyData::testPointCloud()->cloudSource);
    probb.setInputTarget(readPlyData::testPointCloud()->cloudTarget);

    oa::Problem prob{probb};
    oa::Population pop{prob, 25};
    oa::multipleClassTeachingLearningBasedOptimization mctlbo(8, 5, 4);
    //  oa::teachingLearningBasedOptimization mctlbo(20);

    auto res = mctlbo.optimize(pop);
    //  oa::teachingLearningBasedOptimization mctlbo(20);
  }
}

BENCHMARK(BM_MCTLBO)->Unit(benchmark::kMillisecond);