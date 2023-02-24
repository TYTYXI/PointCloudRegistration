#include "FMBF2.h"

using namespace oa;

FMBF2::FMBF2(int n)
    : n_(n)
{
}

FMBF2::~FMBF2()
{
}

oa::VectorDouble oa::FMBF2::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF2::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF2::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF2::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF2::correspondenceEstimation()
{
  return 0.000001;
}
