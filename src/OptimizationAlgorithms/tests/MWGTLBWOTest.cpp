//
// Created by XI on 2023/2/28.
//

#include "../algorithm/WhaleOptimization.h"
#include "../problems/test1Problem.h"
#include "../problems/test2Problem.h"
#include "../problems/UnimodalBenchmarkFunctions/UBF1.h"

#include "algorithm/MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn.h"
#include "benchmark/benchmark.h"
#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include "problems.h"

#define TEST_MACRO(name, index, problem, numOfVars)                                                \
  TEST(name, test##index)                                                                          \
  {                                                                                                \
    oa::Problem prob{oa::problem(numOfVars)};                                                      \
    oa::MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn woa(64, 8, 4);                    \
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

TEST_MACRO(MWGTLBWOA, 1, UBF1, 30)
TEST_MACRO(MWGTLBWOA, 2, UBF2, 30)
TEST_MACRO(MWGTLBWOA, 3, UBF3, 30)
TEST_MACRO(MWGTLBWOA, 4, UBF4, 30)
TEST_MACRO(MWGTLBWOA, 5, UBF5, 30)
TEST_MACRO(MWGTLBWOA, 6, UBF6, 30)
TEST_MACRO(MWGTLBWOA, 7, UBF7, 30)
TEST_MACRO(MWGTLBWOA, 8, MBF1, 30)
TEST_MACRO(MWGTLBWOA, 9, MBF2, 30)
TEST_MACRO(MWGTLBWOA, 10, MBF3, 30)
TEST_MACRO(MWGTLBWOA, 11, MBF4, 30)
TEST_MACRO(MWGTLBWOA, 12, MBF5, 30)
TEST_MACRO(MWGTLBWOA, 13, MBF6, 30)
TEST_MACRO(MWGTLBWOA, 14, FMBF1, 2)
TEST_MACRO(MWGTLBWOA, 15, FMBF2, 4)
TEST_MACRO(MWGTLBWOA, 16, FMBF3, 2)
TEST_MACRO(MWGTLBWOA, 17, FMBF4, 2)
TEST_MACRO(MWGTLBWOA, 18, FMBF5, 2)
TEST_MACRO(MWGTLBWOA, 19, FMBF6, 3)
TEST_MACRO(MWGTLBWOA, 20, FMBF7, 6)
TEST_MACRO(MWGTLBWOA, 21, FMBF8, 4)
TEST_MACRO(MWGTLBWOA, 22, FMBF9, 4)
TEST_MACRO(MWGTLBWOA, 23, FMBF10, 4)
