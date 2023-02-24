#include "FMBF6.h"

using namespace oa;

FMBF6::FMBF6(int n)
    : n_(n)
{
}

FMBF6::~FMBF6()
{
}

oa::VectorDouble oa::FMBF6::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF6::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF6::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF6::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF6::correspondenceEstimation()
{
  return 0.000001;
}
