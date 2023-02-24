#include "FMBF10.h"

using namespace oa;

FMBF10::FMBF10(int n)
    : n_(n)
{
}

FMBF10::~FMBF10()
{
}

oa::VectorDouble oa::FMBF10::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
}

VectorDouble::size_type oa::FMBF10::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF10::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF10::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF10::correspondenceEstimation()
{
  return 0.000001;
}
