#include "FMBF1.h"

using namespace oa;

FMBF1::FMBF1(int n)
    : n_(n)
{
}

FMBF1::~FMBF1()
{
}

oa::VectorDouble oa::FMBF1::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF1::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF1::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF1::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF1::correspondenceEstimation()
{
  return 0.000001;
}
