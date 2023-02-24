//
// Created by XI on 2023/2/23.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_WHALEOPTIMIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_WHALEOPTIMIZATION_H_

#include <iostream>

#include "AlgorithmObject.h"

#include "../Population.hpp"

namespace oa {

class ALGORITHM_EXPORT WhaleOptimization
{

public:
  explicit WhaleOptimization(size_t iteration);

  Population optimize(Population pop);

private:
  size_t iteration_;
};
} // namespace oa

#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_WHALEOPTIMIZATION_H_