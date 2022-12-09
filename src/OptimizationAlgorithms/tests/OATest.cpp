//
// Created by XI on 2022/11/30.
//
#include "../algorithm/ArtificialBeeColony.h"
// #include "MultipleClassTeachingLearningBasedOptimization.h"
#include "../algorithm/TeachingLearningBasedOptimization.h"
#include "../problems/test1Problem.h"

#include "benchmark/benchmark.h"
#include "gtest/gtest.h"

#include <Eigen/Eigen>

TEST(TLBO, test2)
{
  oa::Problem prob{oa::test1Problem()};
  oa::Population pop{prob,30};

  oa::teachingLearningBasedOptimization tlbo(20);

  auto res = tlbo.optimize(pop);

  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test1)
{
  auto b = new float[6];
  for (size_t i = 0; i < 3; ++i) {
    b[2 * i] = -1.;
    b[2 * i + 1] = 1.;
  }
  //  ioa::multipleClassTeachingLearningBasedOptimization<L2_Simple<float>> mctlbo(20, 6, 20);
  //  mctlbo.setLimits(b);
  //  mctlbo.optimize();
  //  float params[3];
  //  mctlbo.parameters(params);
  //  std::cout << params[0] << "  " << params[1] << "  " << params[2] << std::endl;
  //  ASSERT_TRUE(true);
}

TEST(ABC, test1)
{
}