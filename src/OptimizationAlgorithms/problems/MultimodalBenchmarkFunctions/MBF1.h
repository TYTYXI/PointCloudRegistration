//
// Created by XI on 2023/2/24.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_MULTIMODALBENCHMARKFUNCTIONS_MBF1_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_MULTIMODALBENCHMARKFUNCTIONS_MBF1_H_

#include "Problems.hpp"

namespace oa {
class ALGORITHM_EXPORT MBF1
{
public:
  MBF1(int n);
  virtual ~MBF1();
  VectorDouble fitnessScore(const VectorDouble& dv);
  std::pair<VectorDouble, VectorDouble> bounds();
  VectorDouble::size_type dimension();
  VectorDouble::size_type numOfObjectiveFunction();
  VectorDouble::value_type correspondenceEstimation();
private:
  int n_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_MULTIMODALBENCHMARKFUNCTIONS_MBF1_H_