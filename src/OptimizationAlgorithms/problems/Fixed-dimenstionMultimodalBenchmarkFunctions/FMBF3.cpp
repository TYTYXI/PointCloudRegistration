#include "FMBF3.h"

using namespace oa;

FMBF3::FMBF3(int n)
    : n_(n)
{
}

FMBF3::~FMBF3()
{
}

oa::VectorDouble oa::FMBF3::fitnessScore(const oa::VectorDouble& dv)
{

  double sum = 0.;
  sum = 4 * std::pow(dv[0], 2) - 2.1 * std::pow(dv[0], 4) + 1. / 3. * std::pow(dv[0], 6) +
        dv[0] * dv[1] - 4. * std::pow(dv[1], 2) + 4. * std::pow(dv[1], 4);

  return {sum};
}

VectorDouble::size_type oa::FMBF3::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF3::bounds()
{
  VectorDouble lb(n_, -5);
  VectorDouble ub(n_, 5);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF3::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF3::correspondenceEstimation()
{
  return 0.000001;
}
