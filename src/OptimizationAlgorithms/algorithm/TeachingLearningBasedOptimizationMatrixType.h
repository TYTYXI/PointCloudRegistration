//
// Created by XI on 2023/6/19.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATIONMATRIXTYPE_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATIONMATRIXTYPE_H_

#include <iostream>

#include "AlgorithmObject.h"

#include "PopulationMatrixType.hpp"

namespace oa {

class ALGORITHM_EXPORT TeachingLearningBasedOptimizationMatrixType
{
public:
  explicit TeachingLearningBasedOptimizationMatrixType(size_t iteration);

  PopulationMatrixType optimize(oa::PopulationMatrixType pop);

private:
  size_t iteration_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATIONMATRIXTYPE_H_
