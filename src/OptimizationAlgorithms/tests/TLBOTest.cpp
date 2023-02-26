//
// Created by XI on 2023/2/24.
//
#include "../algorithm/WhaleOptimization.h"
#include "../problems/test1Problem.h"
#include "../problems/test2Problem.h"
#include "../problems/UnimodalBenchmarkFunctions/UBF1.h"

#include "algorithm/MultipleClassTeachingLearningBasedOptimization.h"
#include "algorithm/TeachingLearningBasedOptimization.h"
#include "benchmark/benchmark.h"

#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include "problems.h"

#define TEST_MACRO(name, index, prob)                                                              \
  TEST(name, test##index)                                                                          \
  {                                                                                                \
    oa::Problem prob{oa::prob(30)};                                                                \
    oa::teachingLearningBasedOptimization tlbo(500);                                               \
    double avg = 0;                                                                                \
    for (size_t i = 0; i < 30; ++i) {                                                              \
      oa::Population pop{prob, 30};                                                                \
      auto res = tlbo.optimize(pop);                                                               \
      avg += res.championFitnessScores()[0];                                                       \
    }                                                                                              \
                                                                                                   \
    std::cout << avg / 30 << std::endl;                                                            \
    ASSERT_TRUE(true);                                                                             \
  }

TEST(TLBO, test1)
{
  oa::Problem prob{oa::UBF1(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(TLBO, test2)
{
  oa::Problem prob{oa::UBF2(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(TLBO, test3)
{
  oa::Problem prob{oa::UBF3(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(TLBO, test4)
{
  oa::Problem prob{oa::UBF4(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(TLBO, test5)
{
  oa::Problem prob{oa::UBF5(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(TLBO, test6)
{
  oa::Problem prob{oa::UBF6(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(TLBO, test7)
{
  oa::Problem prob{oa::UBF7(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST(TLBO, test8)
{
  oa::Problem prob{oa::MBF1(30)};
  oa::Population pop{prob, 30};
  oa::teachingLearningBasedOptimization tlbo(500);

  auto res = tlbo.optimize(pop);
  std::cout << res.championFitnessScores()[0] << std::endl;

  //  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
  //            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}

TEST_MACRO(TLBO, 12, MBF5)