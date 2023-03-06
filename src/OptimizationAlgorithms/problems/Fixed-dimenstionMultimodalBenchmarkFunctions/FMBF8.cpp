#include "FMBF8.h"

using namespace oa;

FMBF8::FMBF8(int n)
    : n_(n)
{
}

FMBF8::~FMBF8()
{
}

oa::VectorDouble oa::FMBF8::fitnessScore(const oa::VectorDouble& dv)
{
  double aSH[10][4] = {{4, 4, 4, 4}, {1, 1, 1, 1}, {8, 8, 8, 8}, {6, 6, 6, 6}, {3, 7, 3, 7},
                       {2, 9, 2, 9}, {5, 5, 3, 3}, {8, 1, 8, 1}, {6, 2, 6, 2}, {7, 3.6, 7, 3.6}};

  double cSH[] = {.1, .2, .2, .4, .4, .6, .3, .7, .5, .5};
  double A1 = 0.;
  for (size_t i = 0; i < 5; ++i) {
    VectorDouble temp(4, 0);
    for (size_t j = 0; j < 4; ++j) {
      temp[j] = std::pow(dv[j] - aSH[i][j], 2);
    }
    A1 += std::pow(temp[0] + temp[1] + temp[2] + temp[3] + cSH[i], -1);
  }
  return {-A1};
}

VectorDouble::size_type oa::FMBF8::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF8::bounds()
{
  VectorDouble lb(n_, -100);
  VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF8::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF8::correspondenceEstimation()
{
  return 0.000001;
}
