#include "UBF4.h"

using namespace oa;

oa::VectorDouble oa::UBF4::fitnessScore(const oa::VectorDouble& dv)
{
  VectorDouble temp;
  for (size_t i = 0; i < dv.size(); ++i) {
    temp.emplace_back(std::abs(dv[i]));
  }
  auto aa = *std::max_element(temp.cbegin(), temp.cend());
  return {aa};
}

VectorDouble::size_type oa::UBF4::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF4::bounds()
{
  const VectorDouble lb(n_, -100);
  const VectorDouble ub(n_, 100);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF4::numOfObjectiveFunction()
{
  return 1;
}

UBF4::UBF4(int n)
    : n_(n)
{
//  VectorDouble aaa{
//      -18.2674252467276, -42.9771221667240, 49.6552665408588,  23.5905795905667,  -32.0694448995233,
//      -13.5489251495711, 24.2730296056398,  -48.6086180880841, -74.8169936130852, 41.8686564613561,
//      -42.3608261783202, 62.2022427358435,  -10.2183628408983, 10.4487151822364,  -1.06653703067650,
//      45.3816025187255,  -40.1612198829340, 11.1214233009802,  -98.2512481018972, -86.6719856970402,
//      37.4215354842997,  9.67382992600152,  -85.5180978817298, 23.2363171170671,  73.3956209697809,
//      66.2473870125712,  33.0087680403778,  69.9239151648090,  19.9482452264736,  83.3554278086002};
//  auto bbb= fitnessScore(aaa);
}

UBF4::~UBF4()
{
}

VectorDouble::value_type UBF4::correspondenceEstimation()
{
  return 0.000001;
}
