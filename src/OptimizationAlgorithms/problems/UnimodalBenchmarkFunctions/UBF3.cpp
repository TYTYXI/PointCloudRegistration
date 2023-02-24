#include "UBF3.h"

#include <numeric>
using namespace oa;

oa::VectorDouble oa::UBF3::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  for (size_t i = 0; i < n_; ++i) {
    double sum2 = 0;
    for (size_t j = 0; j < i + 1; ++j) {
      sum2 += dv[j];
    }
    sum += std::pow(sum2, 2);
  }
  return {sum};
}

VectorDouble::size_type oa::UBF3::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF3::bounds()
{
  const VectorDouble lb(n_, -100);
  const VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF3::numOfObjectiveFunction()
{
  return 1;
}

UBF3::UBF3(int n)
    : n_(n)
{
}

UBF3::~UBF3()
{
}

VectorDouble::value_type UBF3::correspondenceEstimation()
{
  return 0.000001;
}
