//
// Created by XI on 2022/11/29.
//

#include "gtest/gtest.h"

#include <Eigen/Eigen>

template <class T>
struct L2_Simple
{
  typedef bool is_kdtree_distance;

  typedef T ResultType;

  ResultType operator()(ResultType* a) const
  {
    auto result = (a[0] + a[1]) * (a[0] + a[1]);
    return result;
  }
};