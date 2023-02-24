//
// Created by XI on 2022/12/8.
//

#include "NullProblem.h"

using namespace oa;

oa::VectorDouble oa::NullProblem::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::NullProblem::dimension()
{
  return 0;
}

std::pair<VectorDouble, VectorDouble> oa::NullProblem::bounds()
{
  return std::pair<VectorDouble, VectorDouble>();
}

VectorDouble::size_type oa::NullProblem::numOfObjectiveFunction()
{
  return 0;
}

NullProblem::NullProblem()
{
}

NullProblem::~NullProblem()
{
}

VectorDouble::value_type NullProblem::correspondenceEstimation()
{
  return 0.000001;
}
