//
// Created by XI on 2022/12/8.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_NULLPROBLEM_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_NULLPROBLEM_H_

#include "../Problems.hpp"

namespace oa {
class ALGORITHM_EXPORT NullProblem
{
public:
    NullProblem();
  virtual ~NullProblem();
  VectorDouble fitnessScore(const VectorDouble& dv);
  std::pair<VectorDouble, VectorDouble> bounds();
  VectorDouble::size_type dimension();
  VectorDouble::size_type numOfObjectiveFunction();
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_NULLPROBLEM_H_
