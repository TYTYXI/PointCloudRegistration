#include "FMBF9.h"

using namespace oa;

FMBF9::FMBF9(int n)
    : n_(n)
{
}

FMBF9::~FMBF9()
{
}

oa::VectorDouble oa::FMBF9::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF9::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF9::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF9::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF9::correspondenceEstimation()
{
  return 0.000001;
}
