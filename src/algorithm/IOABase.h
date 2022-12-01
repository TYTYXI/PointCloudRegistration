//
// Created by XI on 2022/11/29.
//

#ifndef POINTCLOUDREGISTRATION_SRC_ALGORITHM_IOABASE_H_
#define POINTCLOUDREGISTRATION_SRC_ALGORITHM_IOABASE_H_

#include "algorithm_global.h"

#include <ctime>
#include <iostream>
#include <random>

namespace ioa {

template <typename ParamsType, int numOfParam, typename TargetFunc>
class ALGORITHM_EXPORT IOABase
{
public:
  IOABase()
  {
  }

  IOABase(int iteration, int numOfParams, TargetFunc targetFunc = TargetFunc())
      : iteration_(iteration)
      , numOfParams_(numOfParams)
      , targetFunc_(targetFunc)
  {
  }

  virtual ~IOABase() = default;

  virtual bool subjectToConstrains(ParamsType* params)
  {
    for (size_t i = 0; i < this->numOfParams_; ++i) {
      if (*params < limits_[2 * i] || *params++ > limits_[2 * i + 1]) {
        return false;
      }
    }
    return true;
  }

  void setMaxIteration(int iteration)
  {
    iteration_ = iteration;
  }

  void setLimits(ParamsType* limits)
  {
    for (size_t i = 0; i < numOfParam; ++i) {
      limits_[2 * i] = *limits++;
      limits_[2 * i + 1] = *limits++;
    }
  }

  virtual void optimize(){};

  virtual void init(ParamsType* params)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    if (std::is_same<ParamsType, int>::value == true) {
      for (size_t i = 0; i < this->numOfParams_; ++i) {
        std::uniform_int_distribution<int> u(limits_[2 * i], limits_[2 * i + 1]);
        params[i] = u(gen);
      }
    } else {
      for (size_t i = 0; i < this->numOfParams_; ++i) {
        std::uniform_real_distribution<ParamsType> u(limits_[2 * i], limits_[2 * i + 1]);
        params[i] = u(gen);
        //        std::cout << params_[i] << std::endl;
      }
    }
  }

protected:
  size_t iteration_;
  size_t numOfParams_;

  double fitnessScore_;

  TargetFunc targetFunc_;

  ParamsType params_[numOfParam];
  ParamsType limits_[2 * numOfParam];
};
} // namespace ioa
#endif // POINTCLOUDREGISTRATION_SRC_ALGORITHM_IOABASE_H_
