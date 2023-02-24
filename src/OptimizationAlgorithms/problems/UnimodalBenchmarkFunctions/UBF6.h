//
// Created by XI on 2023/2/23.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_UNIMODALBENCHMARKFUNCTIONS_UBF6_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_UNIMODALBENCHMARKFUNCTIONS_UBF6_H_

#include "../Problems.hpp"

namespace oa {
class ALGORITHM_EXPORT UBF6
{
public:
  UBF6(int n);
  virtual ~UBF6();
  VectorDouble fitnessScore(const VectorDouble& dv);
  std::pair<VectorDouble, VectorDouble> bounds();
  VectorDouble::size_type dimension();
  VectorDouble::size_type numOfObjectiveFunction();
  VectorDouble::value_type correspondenceEstimation();private:
  int n_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_UNIMODALBENCHMARKFUNCTIONS_UBF6_H_