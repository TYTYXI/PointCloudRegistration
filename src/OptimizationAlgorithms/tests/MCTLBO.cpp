//
// Created by XI on 2023/2/64.
//
#include "../algorithm/WhaleOptimization.h"
#include "../problems/test1Problem.h"
#include "../problems/test2Problem.h"
#include "../problems/UnimodalBenchmarkFunctions/UBF1.h"

#include "algorithm/MultipleClassTeachingLearningBasedOptimization.h"
#include "benchmark/benchmark.h"

#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include "problems.h"

#define TEST_MACRO(name, index, prob)                                                              \
  TEST(name, test##index)                                                                          \
  {                                                                                                \
    oa::Problem prob{oa::prob(30)};                                                                \
    oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);                           \
    double avg = 0;                                                                                \
    for (size_t i = 0; i < 30; ++i) {                                                              \
      oa::Population pop{prob, 30};                                                                \
      auto res = mctlbo.optimize(pop);                                                             \
      avg += res.championFitnessScores()[0];                                                       \
    }                                                                                              \
                                                                                                   \
    std::cout << avg / 30 << std::endl;                                                            \
    ASSERT_TRUE(true);                                                                             \
  }

TEST(MCTLBO, test1)
{
  oa::Problem prob{oa::UBF1(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test2)
{
  oa::Problem prob{oa::UBF2(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test3)
{
  oa::Problem prob{oa::UBF3(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test4)
{
  oa::Problem prob{oa::UBF4(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test5)
{
  oa::Problem prob{oa::UBF5(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test6)
{
  oa::Problem prob{oa::UBF6(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test7)
{
  oa::Problem prob{oa::UBF7(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test8)
{
  oa::Problem prob{oa::MBF1(30)};
  oa::Population pop{prob, 30};
  oa::multipleClassTeachingLearningBasedOptimization mctlbo(64, 8, 4);

  auto res = mctlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST_MACRO(MCTLBO, 12, MBF5)