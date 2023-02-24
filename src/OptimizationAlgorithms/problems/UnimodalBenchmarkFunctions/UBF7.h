//
// Created by XI on 2023/2/23.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_UNIMODALBENCHMARKFUNCTIONS_UBF7_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_UNIMODALBENCHMARKFUNCTIONS_UBF7_H_

#include "../Problems.hpp"

namespace oa {
class ALGORITHM_EXPORT UBF7
{
public:
  UBF7(int n);
  virtual ~UBF7();
  VectorDouble fitnessScore(const VectorDouble& dv);
  std::pair<VectorDouble, VectorDouble> bounds();
  VectorDouble::size_type dimension();
  VectorDouble::size_type numOfObjectiveFunction();
  VectorDouble::value_type correspondenceEstimation();private:
  int n_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_UNIMODALBENCHMARKFUNCTIONS_UBF7_H_