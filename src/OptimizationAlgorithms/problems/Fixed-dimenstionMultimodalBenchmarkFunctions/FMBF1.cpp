#include "FMBF1.h"

using namespace oa;

FMBF1::FMBF1(int n)
    : n_(n)
{
}

FMBF1::~FMBF1()
{
}

oa::VectorDouble oa::FMBF1::fitnessScore(const oa::VectorDouble& dv)
{
  double aS[2][25] = {{-32, -16, 0,   16,  32, -32, -16, 0,   16,  32, -32, -16, 0,
                       16,  32,  -32, -16, 0,  16,  32,  -32, -16, 0,  16,  32},
                      {-32, -32, -32, -32, -32, -16, -16, -16, -16, -16, 0,  0, 0,
                       0,   0,   16,  16,  16,  16,  16,  32,  32,  32,  32, 32}};

  double A1 = 0., A2 = 0.;
  for (size_t i = 0; i < 25; ++i) {
    A1 += 1. / ((double)i + 1 + std::pow(dv[0] - aS[0][i], 6) + std::pow(dv[1] - aS[1][i], 6));
  }
  double sum = 0.;
  sum = std::pow(1. / 500. + A1, -1);

  return {sum};
}

VectorDouble::size_type oa::FMBF1::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF1::bounds()
{
  VectorDouble lb(n_, -65);
  VectorDouble ub(n_, 65);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF1::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF1::correspondenceEstimation()
{
  return 0.000001;
}
