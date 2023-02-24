//
// Created by XI on 2023/2/22.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_CUCKOOSEARCH_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_CUCKOOSEARCH_H_
#include <iostream>

#include "AlgorithmObject.h"

#include "../Population.hpp"

namespace oa {

class ALGORITHM_EXPORT CuckooSearch
{

public:
  explicit CuckooSearch(size_t iteration, size_t numOfClasses, size_t interval);

  Population optimize(Population pop);

private:
  size_t iteration_;
  size_t numOfClasses_;
  size_t interval_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_CUCKOOSEARCH_H_
