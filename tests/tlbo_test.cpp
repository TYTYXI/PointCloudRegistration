//
// Created by XI on 2022/11/29.
//

#include "src/algorithm/TeachingLearningBasedOptimization.h"
#include "gtest/gtest.h"

#include <Eigen/Eigen>

template <class T>
struct TargetFunc
{
  typedef bool is_kdtree_distance;

  typedef T ResultType;

  ResultType operator()(ResultType* a) const
  {
    auto result = (a[0] + a[1]) * (a[0] + a[1]);
    return result;
  }
};

TEST(TLBO, test1)
{
  int aaaa = 1;

  float* b = new float[4];
  for (size_t i = 0; i < 2; ++i) {
    b[2 * i] = -1.;
    b[2 * i + 1] = 1.;
  }
  ioa::teachingLearningBasedOptimization<float, 2, TargetFunc<float>> tlbo(10, 20);
  tlbo.setLimits(b);
  tlbo.optimize();
  ASSERT_TRUE(true);
}