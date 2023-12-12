//
// Created by XI on 2023/2/24.
//
// STD Includes
#include <numeric>
#include <utility>

#include "../algorithm/WhaleOptimization.h"
#include "../problems/test1Problem.h"
#include "../problems/test2Problem.h"
#include "../problems/UnimodalBenchmarkFunctions/UBF1.h"

#include "algorithm/MultipleClassTeachingLearningBasedOptimization.h"
#include "algorithm/TeachingLearningBasedOptimization.h"
#include "algorithm/SmallGroupTeachingLearningBasedOptimization.h"
#include "benchmark/benchmark.h"

#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include "problems.h"

#define TEST_MACRO(name, index, prob, numOfVars)                                                   \
  TEST(name, test##index)                                                                          \
  {                                                                                                \
    oa::Problem prob{oa::prob(numOfVars)};                                                         \
    oa::smallGroupTeachingLearningBasedOptimization tlbo(500);                                               \
    oa::VectorDouble temp;                                                                         \
    std::vector<double> std;                                                                       \
    double avg = 0;                                                                                \
    for (size_t i = 0; i < 30; ++i) {                                                              \
      oa::Population pop{prob, 30};                                                                \
      auto res = tlbo.optimize(pop);                                                               \
      avg += res.championFitnessScores()[0];                                                       \
      temp.emplace_back(res.championFitnessScores()[0]);                                           \
      std.emplace_back(res.championFitnessScores()[0]);                                            \
    }                                                                                              \
                                                                                                   \
    std::cout << "F" << index << "  =  " << avg / 30 << std::endl;                                 \
    std::cout << *std::min_element(temp.cbegin(), temp.cend()) << std::endl;                       \
                                                                                                   \
    for (int i = 0; i < std.size(); i++) {                                                         \
      std[i] = std::pow(std[i] - avg / 30, 2);                                                     \
    }                                                                                              \
    double std_sum = 0.0;                                                                          \
    std_sum = std::accumulate(std.begin(), std.end(), 0.0);                                        \
    std::cout << std::pow(std_sum / 30, 0.5) << std::endl;                                         \
    ASSERT_TRUE(true);                                                                             \
  }

TEST_MACRO(TLBO, 1, UBF1, 30)
TEST_MACRO(TLBO, 2, UBF2, 30)
TEST_MACRO(TLBO, 3, UBF3, 30)
TEST_MACRO(TLBO, 4, UBF4, 30)
TEST_MACRO(TLBO, 5, UBF5, 30)
TEST_MACRO(TLBO, 6, UBF6, 30)
TEST_MACRO(TLBO, 7, UBF7, 30)
TEST_MACRO(TLBO, 8, MBF1, 30)
TEST_MACRO(TLBO, 9, MBF2, 30)
TEST_MACRO(TLBO, 10, MBF3, 30)
TEST_MACRO(TLBO, 11, MBF4, 30)
TEST_MACRO(TLBO, 12, MBF5, 30)
TEST_MACRO(TLBO, 13, MBF6, 30)
TEST_MACRO(TLBO, 14, FMBF1, 2)
TEST_MACRO(TLBO, 15, FMBF2, 4)
TEST_MACRO(TLBO, 16, FMBF3, 2)
TEST_MACRO(TLBO, 17, FMBF4, 2)
TEST_MACRO(TLBO, 18, FMBF5, 2)
TEST_MACRO(TLBO, 19, FMBF6, 3)
TEST_MACRO(TLBO, 20, FMBF7, 6)
TEST_MACRO(TLBO, 21, FMBF8, 4)
TEST_MACRO(TLBO, 22, FMBF9, 4)
TEST_MACRO(TLBO, 23, FMBF10, 4)