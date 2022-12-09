//
// Created by XI on 2022/11/29.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_ALGORITHMOBJECT_H_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_ALGORITHMOBJECT_H_

// OA Includes
#include "OptimizationAlgorithmsGlobal.h"
#include "Types.h"

// STL Includes
#include <random>

namespace oa {

class ALGORITHM_EXPORT AlgorithmObject
{

public:
  AlgorithmObject(size_t iteration)
      : iteration_(iteration)
  {
  }

  virtual ~AlgorithmObject() = default;


  void setSeed(int seed);


  void setMaxIteration(int iteration)
  {
    iteration_ = iteration;
  }

  virtual void optimize(){};

//  virtual void init(ParamsType* params)
//  {
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    if (std::is_same<ParamsType, int>::value == true) {
//      for (size_t i = 0; i < this->numOfParams_; ++i) {
//        std::uniform_int_distribution<int> u(limits_[2 * i], limits_[2 * i + 1]);
//        params[i] = u(gen);
//      }
//    } else {
//      for (size_t i = 0; i < this->numOfParams_; ++i) {
//        std::uniform_real_distribution<ParamsType> u(limits_[2 * i], limits_[2 * i + 1]);
//        params[i] = u(gen);
//        //        std::cout << params_[i] << std::endl;
//      }
//    }
//  }

protected:
  size_t iteration_;

  double fitnessScore_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_ALGORITHM_IOABASE_H_
