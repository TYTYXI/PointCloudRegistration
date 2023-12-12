//
// Created by XI on 2023/6/19.
//
#include "../algorithm/WhaleOptimization.h"
#include "../problems/test1Problem.h"
#include "../problems/test2Problem.h"
#include "../problems/UnimodalBenchmarkFunctions/UBF1.h"

#include "algorithm/TeachingLearningBasedOptimizationMatrixType.h"
#include "benchmark/benchmark.h"

#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include "problems.h"

TEST(name, testTLBOMatrix)
{
  oa::Problem prob{oa::UBF1(20)};
  oa::TeachingLearningBasedOptimizationMatrixType tlbo(500);
  oa::VectorDouble temp;
  double avg = 0;
  for (size_t i = 0; i < 30; ++i) {
    oa::PopulationMatrixType pop{prob, 30};
    auto res = tlbo.optimize(pop);
//    avg += res.championFitnessScores()[0];
//    temp.emplace_back(res.championFitnessScores()[0]);
  }

//  std::cout << avg / 30 << std::endl;
//  std::cout << *std::min_element(temp.cbegin(), temp.cend()) << std::endl;
  ASSERT_TRUE(true);
}