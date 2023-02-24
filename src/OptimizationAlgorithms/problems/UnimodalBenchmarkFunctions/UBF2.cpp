#include "UBF2.h"

using namespace oa;

oa::VectorDouble oa::UBF2::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  for (size_t i = 0; i < n_; ++i) {
    sum += std::abs(dv[i]);
  }

  double temp = 1;
  for (size_t i = 0; i < n_; ++i) {
    temp *= std::abs(dv[i]);
  }

  return {sum};
}

VectorDouble::size_type oa::UBF2::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF2::bounds()
{
    VectorDouble lb(n_, -10);
  VectorDouble ub(n_, 10);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF2::numOfObjectiveFunction()
{
  return 1;
}

UBF2::UBF2(int n)
    : n_(n)
{
}

UBF2::~UBF2()
{
}

VectorDouble::value_type UBF2::correspondenceEstimation()
{
  return 0.000001;
}
