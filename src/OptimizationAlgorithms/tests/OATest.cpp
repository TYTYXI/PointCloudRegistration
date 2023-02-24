//
// Created by XI on 2022/11/30.
//
#include "../algorithm/ArtificialBeeColony.h"
// #include "MultipleClassTeachingLearningBasedOptimization.h"
#include "../algorithm/TeachingLearningBasedOptimization.h"
#include "../algorithm/WhaleOptimization.h"
#include "../problems/test1Problem.h"
#include "../problems/test2Problem.h"

#include "algorithm/MultipleClassTeachingLearningBasedOptimization.h"
#include "benchmark/benchmark.h"
#include "gtest/gtest.h"

#include <Eigen/Eigen>

//TEST(TLBO, test1)
//{
//  oa::Problem prob{oa::test1Problem()};
//  oa::Population pop{prob, 30};
//
//  oa::teachingLearningBasedOptimization tlbo(20);
//
//  auto res = tlbo.optimize(pop);
//
//  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
//            << "   " << res.championFitnessScores()[0] << std::endl;
//  ASSERT_TRUE(true);
//}
//
//TEST(TLBO, test2)
//{
//  oa::Problem prob{oa::test2Problem()};
//  oa::Population pop{prob, 35};
//
//  oa::teachingLearningBasedOptimization tlbo(25);
//
//  auto res = tlbo.optimize(pop);
//
//  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
//            << "   " << res.championFitnessScores()[0] << std::endl;
//  ASSERT_TRUE(true);
//}
//
//TEST(MCTLBO, test1)
//{
//  oa::Problem prob{oa::test2Problem()};
//  oa::Population pop{prob, 25};
//  oa::multipleClassTeachingLearningBasedOptimization mctlbo(24, 8, 4);
//
//  auto res = mctlbo.optimize(pop);
//
//  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
//            << "   " << res.championFitnessScores()[0] << std::endl;
//  ASSERT_TRUE(true);
//}
//
TEST(WOA, test111)
{
  oa::Problem prob{oa::test2Problem()};
  oa::Population pop{prob, 25};
  oa::WhaleOptimization woa(24);

  auto res = woa.optimize(pop);

  std::cout << res.championDecisionVariables()[0] << "  " << res.championDecisionVariables()[1]
            << "   " << res.championFitnessScores()[0] << std::endl;
  ASSERT_TRUE(true);
}