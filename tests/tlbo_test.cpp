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