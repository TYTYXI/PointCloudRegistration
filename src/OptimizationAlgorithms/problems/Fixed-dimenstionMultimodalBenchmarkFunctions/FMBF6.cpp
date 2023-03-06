#include "FMBF6.h"
#include "Constants.hpp"

using namespace oa;

FMBF6::FMBF6(int n)
    : n_(n)
{
}

FMBF6::~FMBF6()
{
}

oa::VectorDouble oa::FMBF6::fitnessScore(const oa::VectorDouble& dv)
{
  double aH[4][3] = {{3, 10, 30}, {.1, 10, 35}, {3, 10, 30}, {.1, 10, 35}};
  double cH[] = {1, 1.2, 3, 3.2};
  double pH[4][3] = {
      {.3689, .117, .2673}, {.4699, .4387, .747}, {.1091, .8732, .5547}, {.03815, .5743, .8828}};
  double A1 = 0., A2 = 0.;
  for (size_t i = 0; i < 4; ++i) {
    A2 = 0.;
    for (size_t j = 0; j < 3; ++j) {
      A2 += aH[i][j] * std::pow(dv[j] - pH[i][j], 2);
    }
    A1 += cH[i] * std::pow(oa::e(), -A2);
  }
  return {-A1};
}

VectorDouble::size_type oa::FMBF6::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF6::bounds()
{
  VectorDouble lb(n_,0);
  VectorDouble ub(n_,1);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF6::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF6::correspondenceEstimation()
{
  return 0.000001;
}
