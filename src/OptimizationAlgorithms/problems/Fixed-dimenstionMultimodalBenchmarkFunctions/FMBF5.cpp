#include "FMBF5.h"

using namespace oa;

FMBF5::FMBF5(int n)
    : n_(n)
{
}

FMBF5::~FMBF5()
{
}

oa::VectorDouble oa::FMBF5::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF5::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF5::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF5::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF5::correspondenceEstimation()
{
  return 0.000001;
}
