#include "UBF1.h"

using namespace oa;

oa::VectorDouble oa::UBF1::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  for (size_t i = 0; i < n_; ++i) {
    sum += std::pow(dv[i], 2);
  }
  return {sum};
}

VectorDouble::size_type oa::UBF1::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF1::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF1::numOfObjectiveFunction()
{
  return 1;
}

UBF1::UBF1(int n)
    : n_(n)
{
}

UBF1::~UBF1()
{
}

VectorDouble::value_type UBF1::correspondenceEstimation()
{
  return 0.0000000000001;
}
