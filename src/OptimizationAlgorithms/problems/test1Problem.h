//
// Created by XI on 2022/12/8.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_TEST1PROBLEM_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_TEST1PROBLEM_H_

#include "OptimizationAlgorithmsGlobal.h"
#include "Types.h"

namespace oa {

class ALGORITHM_EXPORT test1Problem
{
public:
  test1Problem();
  virtual ~test1Problem();
  VectorDouble fitnessScore(const oa::VectorDouble& dv);
  VectorDouble::size_type numOfObjectiveFunction();
  VectorDouble::size_type dimension();
  std::pair<oa::VectorDouble, oa::VectorDouble> bounds();
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_TEST1PROBLEM_H_
