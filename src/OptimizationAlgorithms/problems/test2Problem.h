//
// Created by XI on 2022/12/9.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_TEST2PROBLEM_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_TEST2PROBLEM_H_

#include "OptimizationAlgorithmsGlobal.h"
#include "Types.h"

namespace oa {

class ALGORITHM_EXPORT test2Problem
{
public:
  test2Problem();
  virtual ~test2Problem();
  VectorDouble fitnessScore(const oa::VectorDouble& dv);
  VectorDouble::size_type numOfObjectiveFunction();
  VectorDouble::size_type dimension();
  std::pair<oa::VectorDouble, oa::VectorDouble> bounds();
};
}

#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_TEST2PROBLEM_H_
