//
// Created by XI on 2023/2/28.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_MULTIPLEWHALEGROUPSTEACHINGLEARNINGBASEDWHALEOPTIMIZIOTN_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_MULTIPLEWHALEGROUPSTEACHINGLEARNINGBASEDWHALEOPTIMIZIOTN_H_

#include <iostream>

#include "AlgorithmObject.h"

#include "../Population.hpp"

namespace oa {

class ALGORITHM_EXPORT MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn
{

public:
  explicit MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn(size_t iteration,
                                                                    size_t numOfWhaleGroups,
                                                                    size_t interval);

  Population optimize(Population pop);

private:
  size_t iteration_;
  size_t numOfWhaleGroups_;
  size_t interval_;
};
} // namespace oa

#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_MULTIPLEWHALEGROUPSTEACHINGLEARNINGBASEDWHALEOPTIMIZIOTN_H_