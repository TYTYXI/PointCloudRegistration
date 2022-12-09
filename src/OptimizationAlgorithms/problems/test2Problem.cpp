//
// Created by XI on 2022/12/9.
//

#include "test2Problem.h"

#include "Constants.hpp"

using namespace oa;

test2Problem::test2Problem()
{
}

oa::test2Problem::~test2Problem()
{
}

oa::VectorDouble oa::test2Problem::fitnessScore(const oa::VectorDouble& dv)
{
  //  return { (dv[0] * std::cos(2 * oa::pi() * dv[1]) + dv[1] * std::sin(2 * oa::pi() * dv[0]))};
  auto res = (dv[0] * std::cos(2 * oa::pi() * dv[1]) + dv[1] * std::sin(2 * oa::pi() * dv[0]));
  if (res < 0) {
    return {1.};
  } else {
    return {1 / res};
  }
}

VectorDouble::size_type oa::test2Problem::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::size_type oa::test2Problem::dimension()
{
  return 2;
}

std::pair<oa::VectorDouble, oa::VectorDouble> oa::test2Problem::bounds()
{
  return {{-2, -2}, {2, 2}};
}
