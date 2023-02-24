#include "UBF5.h"

using namespace oa;

oa::VectorDouble oa::UBF5::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  for (size_t i = 1; i < dv.size(); ++i) {
    sum += 100 * std::pow(dv[i] - std::pow(dv[i - 1], 2), 2) + std::pow(dv[i - 1], 2);
  }
  return {sum};
}

VectorDouble::size_type oa::UBF5::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF5::bounds()
{
  const VectorDouble lb(n_, -30);
  const VectorDouble ub(n_, 30);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF5::numOfObjectiveFunction()
{
  return 1;
}

UBF5::UBF5(int n)
    : n_(n)
{
}

UBF5::~UBF5()
{
}

VectorDouble::value_type UBF5::correspondenceEstimation()
{
  return 0.000001;
}
