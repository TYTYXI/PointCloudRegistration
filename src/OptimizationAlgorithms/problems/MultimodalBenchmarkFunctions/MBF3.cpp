#include "MBF3.h"
#include "Constants.hpp"

using namespace oa;

MBF3::MBF3(int n)
    : n_(n)
{
}

MBF3::~MBF3()
{
}

oa::VectorDouble oa::MBF3::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  double A1 = 0, A2 = 0;
  for (size_t i = 0; i < dv.size(); ++i) {
    A1 += std::pow(dv[i], 2);
  }
  for (size_t i = 0; i < dv.size(); ++i) {
    A2 += std::cos(2 * oa::pi() * dv[i]);
  }
  sum = -20 * std::pow(oa::e(), -0.2 * std::pow(1.0 / dv.size() * A1, 0.5)) -
        std::pow(oa::e(), 1.0 / dv.size() * A2) + 20 + oa::e();
  return {sum};
}

VectorDouble::size_type oa::MBF3::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::MBF3::bounds()
{
  VectorDouble lb(n_, -32);
  VectorDouble ub(n_, 32);
  return {lb, ub};
}

VectorDouble::size_type oa::MBF3::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type MBF3::correspondenceEstimation()
{
  return 0.000001;
}
