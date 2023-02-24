//
// Created by XI on 2023/2/23.
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
    oa::WhaleOptimization woa(500);                                                                \
    double avg = 0;                                                                                \
    for (size_t i = 0; i < 30; ++i) {                                                              \
      oa::Population pop{prob, 30};                                                                \
      auto res = woa.optimize(pop);                                                                \
      avg += res.championFitnessScores()[0];                                                       \
    }                                                                                              \
                                                                                                   \
    std::cout << avg / 30 << std::endl;                                                            \
    ASSERT_TRUE(true);                                                                             \
  }

TEST_MACRO(WOA, 1, UBF1)
TEST_MACRO(WOA, 2, UBF2)
TEST_MACRO(WOA, 3, UBF3)
TEST_MACRO(WOA, 4, UBF4)
TEST_MACRO(WOA, 5, UBF5)
TEST_MACRO(WOA, 6, UBF6)
TEST_MACRO(WOA, 7, UBF7)
TEST_MACRO(WOA, 8, MBF1)
TEST_MACRO(WOA, 9, MBF2)
TEST_MACRO(WOA, 10, MBF3)
TEST_MACRO(WOA, 11, MBF4)
TEST_MACRO(WOA, 12, MBF5)
TEST_MACRO(WOA, 13, MBF6)
