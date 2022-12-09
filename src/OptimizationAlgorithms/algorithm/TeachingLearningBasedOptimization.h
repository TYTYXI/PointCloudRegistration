//
// Created by XI on 2022/11/29.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATION_H_

#include <iostream>

#include "AlgorithmObject.h"

#include "../Population.hpp"

namespace oa {

class ALGORITHM_EXPORT teachingLearningBasedOptimization
{
public:
  explicit teachingLearningBasedOptimization(size_t iteration);

  Population optimize(Population pop);

private:
  size_t iteration_;
};

} // namespace oa

#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATION_H_
