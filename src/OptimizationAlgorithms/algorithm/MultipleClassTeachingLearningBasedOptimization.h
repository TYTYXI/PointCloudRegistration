//
// Created by XI on 2022/12/6.
//

#ifndef POINTCLOUDREGISTRATION_SRC_ALGORITHM_MULTIPLECLASSTEACHINGLEARNINGBASEDOPTIMIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_ALGORITHM_MULTIPLECLASSTEACHINGLEARNINGBASEDOPTIMIZATION_H_

#include <iostream>

#include "AlgorithmObject.h"

#include "../Population.hpp"

namespace oa {

class ALGORITHM_EXPORT multipleClassTeachingLearningBasedOptimization
{
public:
  explicit multipleClassTeachingLearningBasedOptimization(size_t iteration, size_t numOfClasses);

  Population optimize(Population pop);

private:
  size_t iteration_;
  size_t numOfClasses_;
};



} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_ALGORITHM_MULTIPLECLASSTEACHINGLEARNINGBASEDOPTIMIZATION_H_
