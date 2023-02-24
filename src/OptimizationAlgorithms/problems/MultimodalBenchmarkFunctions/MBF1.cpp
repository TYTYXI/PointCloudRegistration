#include "MBF1.h"

using namespace oa;

MBF1::MBF1(int n)
    : n_(n)
{
}

MBF1::~MBF1()
{
}

oa::VectorDouble oa::MBF1::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  for (size_t i = 0; i < dv.size(); ++i) {
    sum += -1 * dv[i] * std::sin(std::pow(std::abs(dv[i]), 0.5));
  }
  return {sum};
}

VectorDouble::size_type oa::MBF1::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::MBF1::bounds()
{
  VectorDouble lb(n_, -500);
  VectorDouble ub(n_, 500);
  return {lb, ub};
}

VectorDouble::size_type oa::MBF1::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type MBF1::correspondenceEstimation()
{
  return 0.000001;
}
