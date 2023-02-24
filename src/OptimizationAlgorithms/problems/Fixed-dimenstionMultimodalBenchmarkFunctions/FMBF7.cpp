#include "FMBF7.h"

using namespace oa;

FMBF7::FMBF7(int n)
    : n_(n)
{
}

FMBF7::~FMBF7()
{
}

oa::VectorDouble oa::FMBF7::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF7::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF7::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF7::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF7::correspondenceEstimation()
{
  return 0.000001;
}
