//
// Created by XI on 2023/10/16.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_SMALLGROUPTEACHINGLEARNINGBASEDOPTIMIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_SMALLGROUPTEACHINGLEARNINGBASEDOPTIMIZATION_H_

#include <iostream>

#include "AlgorithmObject.h"

#include "../Population.hpp"

namespace oa {

class ALGORITHM_EXPORT smallGroupTeachingLearningBasedOptimization
{
public:
  explicit smallGroupTeachingLearningBasedOptimization(size_t iteration);

  Population optimize(Population pop);

private:
  size_t iteration_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_SMALLGROUPTEACHINGLEARNINGBASEDOPTIMIZATION_H_
