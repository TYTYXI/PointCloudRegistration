#include "FMBF2.h"

using namespace oa;

FMBF2::FMBF2(int n)
    : n_(n)
{
}

FMBF2::~FMBF2()
{
}

oa::VectorDouble oa::FMBF2::fitnessScore(const oa::VectorDouble& dv)
{
  double aK[] = {.1957, .1947, .1735, .16, .0844, .0627, .0456, .0342, .0323, .0235, .0246};
  double bK[] = {4,
                 2,
                 1,
                 0.500000000000000,
                 0.250000000000000,
                 0.166666666666667,
                 0.125000000000000,
                 0.100000000000000,
                 0.0833333333333333,
                 0.0714285714285714,
                 0.0625000000000000};
  double A1 = 0.;
  for (size_t i = 0; i < 11; ++i) {
    A1 += std::pow(aK[i] - ((dv[0] * (std::pow(bK[i], 2) + bK[i] * dv[1])) /
                            ((std::pow(bK[i], 2) + bK[i] * dv[2] + dv[3]))),
                   2);
  }
  double sum = 0.;
  sum = A1;

  return {sum};
}

VectorDouble::size_type oa::FMBF2::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF2::bounds()
{
  VectorDouble lb(n_, -5);
  VectorDouble ub(n_, 5);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF2::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF2::correspondenceEstimation()
{
  return 0.000001;
}
