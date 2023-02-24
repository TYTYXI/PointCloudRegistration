//
// Created by XI on 2022/12/8.
//

#include "test1Problem.h"

#include "Constants.hpp"

using namespace oa;

oa::VectorDouble::size_type test1Problem::dimension()
{
  return 2;
}

std::pair<oa::VectorDouble, oa::VectorDouble> test1Problem::bounds()
{
  return {{-2, -2}, {2, 2}};
}

oa::VectorDouble test1Problem::fitnessScore(const oa::VectorDouble& x)
{
  return {std::pow((x[0] + x[1]), 2)};
}

oa::VectorDouble::size_type test1Problem::numOfObjectiveFunction()
{
  return 1;
}

test1Problem::test1Problem()
{
}

test1Problem::~test1Problem()
{
}

VectorDouble::value_type test1Problem::correspondenceEstimation()
{
  return 0.000001;
}