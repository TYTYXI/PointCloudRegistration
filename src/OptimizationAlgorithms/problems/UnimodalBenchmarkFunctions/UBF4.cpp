#include "UBF4.h"

using namespace oa;

oa::VectorDouble oa::UBF4::fitnessScore(const oa::VectorDouble& dv)
{
  VectorDouble temp;
  for (size_t i = 0; i < dv.size(); ++i) {
    temp.emplace_back(std::abs(dv[i]));
  }
  return {*std::max_element(temp.cbegin(),temp.cend())};
}

VectorDouble::size_type oa::UBF4::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF4::bounds()
{
  const VectorDouble lb(n_, -100);
  const VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF4::numOfObjectiveFunction()
{
  return 1;
}

UBF4::UBF4(int n)
    : n_(n)
{
}

UBF4::~UBF4()
{
}

VectorDouble::value_type UBF4::correspondenceEstimation()
{
  return 0.000001;
}
