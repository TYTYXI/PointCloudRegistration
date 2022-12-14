//
// Created by XI on 2022/12/8.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_CONSTANTS_HPP_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_CONSTANTS_HPP_

#ifdef OA_CXX20
#include <numbers>
#endif

namespace oa {

#ifdef OA_CXX20
constexpr double pi()
{
  return std::numbers::pi;
}

constexpr double pi_half()
{
  return std::numbers::pi / 2.;
}
#elif OA_CXX17
constexpr double pi()
{
  return 3.1415926535897932384626433832795028841971693993;
}

constexpr double piHalf()
{
  return 3.1415926535897932384626433832795028841971693993 / 2.;
}

#endif
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_CONSTANTS_HPP_
