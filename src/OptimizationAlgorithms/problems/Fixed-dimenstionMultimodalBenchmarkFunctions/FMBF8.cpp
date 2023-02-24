#include "FMBF8.h"

using namespace oa;

FMBF8::FMBF8(int n)
    : n_(n)
{
}

FMBF8::~FMBF8()
{
}

oa::VectorDouble oa::FMBF8::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF8::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF8::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF8::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF8::correspondenceEstimation()
{
  return 0.000001;
}
