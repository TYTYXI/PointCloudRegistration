#include "FMBF4.h"
#include "Constants.hpp"

using namespace oa;

FMBF4::FMBF4(int n)
    : n_(n)
{
}

FMBF4::~FMBF4()
{
}

oa::VectorDouble oa::FMBF4::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = std::pow(dv[1] - 5.1 / (4 * std::pow(oa::pi(), 2) )* std::pow(dv[0], 2) +
                            5. / oa::pi() * dv[0] - 6.,
                        2) +
               10 * (1. - 1. / (8 * oa::pi())) * std::cos(dv[0]) + 10;
  return {sum};
}

VectorDouble::size_type oa::FMBF4::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF4::bounds()
{
  VectorDouble lb(n_, -5);
  VectorDouble ub(n_,5);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF4::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF4::correspondenceEstimation()
{
  return 0.000001;
}
