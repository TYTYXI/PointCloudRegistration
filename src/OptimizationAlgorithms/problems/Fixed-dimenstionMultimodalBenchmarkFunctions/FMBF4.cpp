#include "FMBF4.h"

using namespace oa;

FMBF4::FMBF4(int n)
    : n_(n)
{
}

FMBF4::~FMBF4()
{
}

oa::VectorDouble oa::FMBF4::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF4::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF4::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF4::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF4::correspondenceEstimation()
{
  return 0.000001;
}
