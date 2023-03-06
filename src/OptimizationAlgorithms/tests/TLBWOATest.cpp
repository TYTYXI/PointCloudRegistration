//
// Created by XI on 2023/2/28.
//

#include "../algorithm/WhaleOptimization.h"
#include "../problems/test1Problem.h"
#include "../problems/test2Problem.h"
#include "../problems/UnimodalBenchmarkFunctions/UBF1.h"

#include "algorithm/TeachingLearningBasedWhaleOptimiziotn.h"
#include "benchmark/benchmark.h"
#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include "problems.h"

#define TEST_MACRO(name, index, problem, numOfVars)                                                \
  TEST(name, test##index)                                                                          \
  {                                                                                                \
    oa::Problem prob{oa::problem(numOfVars)};                                                      \
    oa::TeachingLearningBasedWhaleOptimiziotn woa(500);                                            \
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

TEST_MACRO(TLBWOA, 1, UBF1, 30)
TEST_MACRO(TLBWOA, 2, UBF2, 30)
TEST_MACRO(TLBWOA, 3, UBF3, 30)
TEST_MACRO(TLBWOA, 4, UBF4, 30)
TEST_MACRO(TLBWOA, 5, UBF5, 30)
TEST_MACRO(TLBWOA, 6, UBF6, 30)
TEST_MACRO(TLBWOA, 7, UBF7, 30)
TEST_MACRO(TLBWOA, 8, MBF1, 30)
TEST_MACRO(TLBWOA, 9, MBF2, 30)
TEST_MACRO(TLBWOA, 10, MBF3, 30)
TEST_MACRO(TLBWOA, 11, MBF4, 30)
TEST_MACRO(TLBWOA, 12, MBF5, 30)
TEST_MACRO(TLBWOA, 13, MBF6, 30)
TEST_MACRO(TLBWOA, 14, FMBF1, 2)
TEST_MACRO(TLBWOA, 15, FMBF2, 4)
TEST_MACRO(TLBWOA, 16, FMBF3, 2)
TEST_MACRO(TLBWOA, 17, FMBF4, 2)
TEST_MACRO(TLBWOA, 18, FMBF5, 2)
TEST_MACRO(TLBWOA, 19, FMBF6, 3)
TEST_MACRO(TLBWOA, 20, FMBF7, 6)
TEST_MACRO(TLBWOA, 21, FMBF8, 4)
TEST_MACRO(TLBWOA, 22, FMBF9, 4)
TEST_MACRO(TLBWOA, 23, FMBF10, 4)