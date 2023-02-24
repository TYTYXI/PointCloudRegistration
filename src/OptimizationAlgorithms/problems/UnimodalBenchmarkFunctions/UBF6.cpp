#include "UBF6.h"

using namespace oa;

oa::VectorDouble oa::UBF6::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  for (size_t i = 0; i < dv.size(); ++i) {
    sum += std::pow(dv[i] + 0.5, 2);
  }
  return {sum};
}

VectorDouble::size_type oa::UBF6::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF6::bounds()
{
  const VectorDouble lb(n_, -100);
  const VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF6::numOfObjectiveFunction()
{
  return 1;
}

UBF6::UBF6(int n)
    : n_(n)
{
}

UBF6::~UBF6()
{
}

VectorDouble::value_type UBF6::correspondenceEstimation()
{
  return 0.000001;
}
