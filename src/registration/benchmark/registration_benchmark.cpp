//
// Created by XI on 2022/12/6.
//

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

// BM TEST Includes
#include "BM_IA_FPCS.h"
#include "BM_IA_KFPCS.h"
#include "BM_IA_RANSAC.h"
#include "BM_ICP_P2N.h"
#include "BM_ICP_P2P.h"
#include "BM_MCTLBO.h"

BENCHMARK_MAIN();

// int main(int argc, char** argv)
//{
//   benchmark::Initialize(&argc, argv);
//   benchmark::RunSpecifiedBenchmarks();
//   benchmark::Shutdown();
// }

static void CustomArguments(benchmark::internal::Benchmark* b)
{
  for (int l = 6; l < 41; ++l) {
    //    if (l % 10 == 1) {
    //      continue;
    //    }
    b->Args({l});
  }
}

// 效率测试
 BENCHMARK(BM_EFFICIENCY_MCTLBO)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(1)
    ->Apply(CustomArguments);

// 准确度测试
//BENCHMARK(BM_ACCURACY_MCTLBO)->Unit(benchmark::kMillisecond)->Iterations(100);
//    ->Apply(CustomArguments);
