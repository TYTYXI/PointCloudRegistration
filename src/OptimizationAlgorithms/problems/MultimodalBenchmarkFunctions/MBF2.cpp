#include "MBF2.h"
#include "Constants.hpp"

using namespace oa;

MBF2::MBF2(int n)
    : n_(n)
{
}

MBF2::~MBF2()
{
}

oa::VectorDouble oa::MBF2::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  for (size_t i = 0; i < dv.size(); ++i) {
    sum += std::pow(dv[i], 2) - 10 * std::cos(2 * oa::pi() * dv[i]) + 10;
  }
  return {sum};
}

VectorDouble::size_type oa::MBF2::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::MBF2::bounds()
{
  VectorDouble lb(n_, -5.12);
  VectorDouble ub(n_, 5.12);
  return {lb, ub};
}

VectorDouble::size_type oa::MBF2::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type MBF2::correspondenceEstimation()
{
  return 0.000001;
}
