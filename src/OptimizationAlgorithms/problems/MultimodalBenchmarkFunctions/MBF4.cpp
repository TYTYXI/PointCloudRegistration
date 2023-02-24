#include "MBF4.h"

using namespace oa;

MBF4::MBF4(int n)
    : n_(n)
{
}

MBF4::~MBF4()
{
}

oa::VectorDouble oa::MBF4::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  double A1 = 0;
  for (size_t i = 0; i < dv.size(); ++i) {
    A1 += std::pow(dv[i], 2);
  }
  double A2 = 1;
  for (size_t i = 0; i < dv.size(); ++i) {
    A2 *= std::cos(dv[i] / std::pow(i+1, 0.5));
  }

  sum = 1.0 / 4000.0 * A1 - A2 + 1;
  return {sum};
}

VectorDouble::size_type oa::MBF4::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::MBF4::bounds()
{
  VectorDouble lb(n_, -600);
  VectorDouble ub(n_, 600);
  return {lb, ub};
}

VectorDouble::size_type oa::MBF4::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type MBF4::correspondenceEstimation()
{
  return 0.000001;
}
