//
// Created by XI on 2022/12/8.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_CONSTANTS_HPP_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_CONSTANTS_HPP_

#include "OptimizationAlgorithmsGlobal.h"

#ifdef OA_CXX20
#include <numbers>
#endif

namespace oa {

#ifdef OA_CXX20
ALGORITHM_EXPORT constexpr double pi()
{
  return std::numbers::pi;
}

ALGORITHM_EXPORT constexpr double pi_half()
{
  return std::numbers::pi / 2.;
}

ALGORITHM_EXPORT constexpr double e()
{
  return std::numbers::e;
}

#else
ALGORITHM_EXPORT constexpr double pi()
{
  return 3.1415926535897932384626433832795028841971693993;
}

ALGORITHM_EXPORT constexpr double piHalf()
{
  return 3.1415926535897932384626433832795028841971693993 / 2.;
}

ALGORITHM_EXPORT constexpr double e()
{
  return 2.7182818284590452354;
}
#endif
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_CONSTANTS_HPP_
