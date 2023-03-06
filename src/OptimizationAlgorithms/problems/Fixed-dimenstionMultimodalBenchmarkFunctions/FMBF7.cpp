#include "FMBF7.h"
#include "Constants.hpp"

using namespace oa;

FMBF7::FMBF7(int n)
    : n_(n)
{
//  VectorDouble aaa = {
//    0.353180341484114, 0.835314665600126, 0.383570359341889, 0.628671556630374, 0.978064721095888,
//    0.873203100511156};
//  auto bbb= fitnessScore(aaa);
}

FMBF7::~FMBF7()
{
}

oa::VectorDouble oa::FMBF7::fitnessScore(const oa::VectorDouble& dv)
{

  double aH[4][6] = {{10, 3, 17, 3.50000000000000, 1.70000000000000, 8},
                     {0.0500000000000000, 10, 17, 0.100000000000000, 8, 14},
                     {3, 3.50000000000000, 1.70000000000000, 10, 17, 8},
                     {17, 8, 0.0500000000000000, 10, 0.100000000000000, 14}};
  double cH[] = {1, 1.2, 3, 3.2};
  double pH[4][6] = {{0.131200000000000, 0.169600000000000, 0.556900000000000, 0.0124000000000000,
                      0.828300000000000, 0.588600000000000},
                     {0.232900000000000, 0.413500000000000, 0.830700000000000, 0.373600000000000,
                      0.100400000000000, 0.999100000000000},
                     {0.234800000000000, 0.141500000000000, 0.352200000000000, 0.288300000000000,
                      0.304700000000000, 0.665000000000000},
                     {0.404700000000000, 0.882800000000000, 0.873200000000000, 0.574300000000000,
                      0.109100000000000, 0.0381000000000000}};
  double A1 = 0., A2 = 0.;
  for (size_t i = 0; i < 4; ++i) {
    A2 = 0.;
    for (size_t j = 0; j < 6; ++j) {
      A2 += aH[i][j] * std::pow(dv[j] - pH[i][j], 2);
    }
    A1 += cH[i] * std::pow(oa::e(), -A2);
  }
  return {-A1};
}

VectorDouble::size_type oa::FMBF7::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::FMBF7::bounds()
{
  VectorDouble lb(n_, 0);
  VectorDouble ub(n_, 1);
  return {lb, ub};
}

VectorDouble::size_type oa::FMBF7::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type FMBF7::correspondenceEstimation()
{
  return 0.000001;
}
