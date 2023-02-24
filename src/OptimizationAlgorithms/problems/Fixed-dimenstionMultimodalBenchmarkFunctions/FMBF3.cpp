#include "FMBF3.h"

using namespace oa;

FMBF3::FMBF3(int n)
    : n_(n)
{
}

FMBF3::~FMBF3()
{
}

oa::VectorDouble oa::FMBF3::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF3::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF3::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF3::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF3::correspondenceEstimation()
{
  return 0.000001;
}
