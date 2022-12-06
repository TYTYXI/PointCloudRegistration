//
// Created by XI on 2022/11/30.
//
#include "ArtificialBeeColony.h"
//#include "MultipleClassTeachingLearningBasedOptimization.h"
#include "TeachingLearningBasedOptimization.h"

#include "benchmark/benchmark.h"
#include "gtest/gtest.h"

#include <Eigen/Eigen>

template <class T>
struct TargetFunc
{
  typedef T ParamsType;
  typedef T ResultType;

  static constexpr int const numOfParams = 3;

  ResultType operator()(ResultType* a) const
  {
    auto result = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    return result;
  }
};

TEST(TLBO, test2)
{
  int aaaa = 1;

  auto b = new float[6];
  for (size_t i = 0; i < 3; ++i) {
    b[2 * i] = -1.;
    b[2 * i + 1] = 1.;
  }
  ioa::teachingLearningBasedOptimization<TargetFunc<float>> tlbo(5, 20);
  tlbo.setLimits(b);
  tlbo.initTLBO();
  for (size_t i = 0; i < 10; ++i) {
    tlbo.optimize();
    float params[3];
    tlbo.parameters(params);
    std::cout << params[0] << "  " << params[1] << "  " << params[2] << std::endl;
  }
  ASSERT_TRUE(true);
}

TEST(MCTLBO, test1)
{
  auto b = new float[6];
  for (size_t i = 0; i < 3; ++i) {
    b[2 * i] = -1.;
    b[2 * i + 1] = 1.;
  }
//  ioa::multipleClassTeachingLearningBasedOptimization<TargetFunc<float>> mctlbo(20, 6, 20);
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