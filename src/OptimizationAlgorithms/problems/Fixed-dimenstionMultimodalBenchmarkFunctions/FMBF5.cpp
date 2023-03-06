#include "FMBF5.h"

using namespace oa;

FMBF5::FMBF5(int n)
    : n_(n)
{
}

FMBF5::~FMBF5()
{
}

oa::VectorDouble oa::FMBF5::fitnessScore(const oa::VectorDouble& dv)
{
  double A1 = 0., A2 = 0.;
  A1 =
      (1 + std::pow(dv[0] + dv[1] + 1, 2) * (19 - 14 * dv[0] + 3 * std::pow(dv[0], 2) - 14 * dv[1] +
                                             6 * dv[0] * dv[1] + 3 * std::pow(dv[1], 2)));
  A2 = 30 + std::pow(2 * dv[0] - 3 * dv[1], 2) *
                (18 - 32 * dv[0] + 12 * std::pow(dv[0], 2) + 48 * dv[1] - 36 * dv[0] * dv[1] +
                 27 * std::pow(dv[1], 2));
  double sum = A1 * A2;
  return {sum};
}

VectorDouble::size_type oa::FMBF5::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF5::bounds()
{
  VectorDouble lb(n_, -2);
  VectorDouble ub(n_,2);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF5::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF5::correspondenceEstimation()
{
  return 0.000001;
}
