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

#define TEST_MACRO(name, index, problem, numOfVars)                                                \
  TEST(name, test##index)                                                                          \
  {                                                                                                \
    oa::Problem prob{oa::problem(numOfVars)};                                                      \
    oa::WhaleOptimization woa(500);                                                                \
    oa::VectorDouble temp;                                                                         \
    double avg = 0;                                                                                \
    for (size_t i = 0; i < 30; ++i) {                                                              \
      oa::Population pop{prob, 30};                                                                \
      auto res = woa.optimize(pop);                                                                \
      avg += res.championFitnessScores()[0];                                                       \
      temp.emplace_back(res.championFitnessScores()[0]);                                           \
    }                                                                                              \
                                                                                                   \
    std::cout << "F" << index << "  =  " << avg / 30 << std::endl;                                 \
    std::cout << *std::min_element(temp.cbegin(), temp.cend()) << std::endl;                       \
    ASSERT_TRUE(true);                                                                             \
  }

TEST_MACRO(WOA, 1, UBF1, 30)
TEST_MACRO(WOA, 2, UBF2, 30)
TEST_MACRO(WOA, 3, UBF3, 30)
TEST_MACRO(WOA, 4, UBF4, 30)
TEST_MACRO(WOA, 5, UBF5, 30)
TEST_MACRO(WOA, 6, UBF6, 30)
TEST_MACRO(WOA, 7, UBF7, 30)
TEST_MACRO(WOA, 8, MBF1, 30)
TEST_MACRO(WOA, 9, MBF2, 30)
TEST_MACRO(WOA, 10, MBF3, 30)
TEST_MACRO(WOA, 11, MBF4, 30)
TEST_MACRO(WOA, 12, MBF5, 30)
TEST_MACRO(WOA, 13, MBF6, 30)
TEST_MACRO(WOA, 14, FMBF1, 2)
TEST_MACRO(WOA, 15, FMBF2, 4)
TEST_MACRO(WOA, 16, FMBF3, 2)
TEST_MACRO(WOA, 17, FMBF4, 2)
TEST_MACRO(WOA, 18, FMBF5, 2)
TEST_MACRO(WOA, 19, FMBF6, 3)
TEST_MACRO(WOA, 20, FMBF7, 6)
TEST_MACRO(WOA, 21, FMBF8, 4)
TEST_MACRO(WOA, 22, FMBF9, 4)
TEST_MACRO(WOA, 23, FMBF10, 4)
