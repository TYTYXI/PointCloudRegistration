//
// Created by XI on 2022/11/30.
//
#include "TeachingLearningBasedOptimization.h"
#include "gtest/gtest.h"

#include <Eigen/Eigen>

template <class T>
struct TargetFunc
{
  typedef T ResultType;

  ResultType operator()(ResultType* a) const
  {
    auto result = (a[0] + a[1]) * (a[0] + a[1]);
    return result;
  }
};

TEST(TLBO, test2)
{
  int aaaa = 1;

  double* b = new double[4];
  for (size_t i = 0; i < 2; ++i) {
    b[2 * i] = -1.;
    b[2 * i + 1] = 1.;
  }
  ioa::teachingLearningBasedOptimization<double, 2, TargetFunc<double>> tlbo(10, 20);
  tlbo.setLimits(b);
  tlbo.optimize();
  double params[2];
  tlbo.parameters(params);
  std::cout << params[0] << "  " << params[1] << std::endl;
  ASSERT_TRUE(true);
}