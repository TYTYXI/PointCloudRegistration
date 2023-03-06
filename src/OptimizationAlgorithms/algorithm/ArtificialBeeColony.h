//
// Created by XI on 2023/3/2.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_ARTIFICIALBEECOLONY_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_ARTIFICIALBEECOLONY_H_

#include <iostream>

#include "AlgorithmObject.h"

#include "../Population.hpp"

namespace oa {

class ALGORITHM_EXPORT ArtificialBeeColony
{

public:
  explicit ArtificialBeeColony(size_t iteration);

  Population optimize(Population pop);

private:
  size_t iteration_;
};
} // namespace oa

#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_ARTIFICIALBEECOLONY_H_