#include "MBF6.h"
#include "Constants.hpp"

using namespace oa;

MBF6::MBF6(int n)
    : n_(n)
{
  VectorDouble aa{
      6.09505409902933, -8.16208163733506, 44.3305681266445,  18.5580867226340,  28.4230448882676,
      33.8675828410127, -12.0179430317296, -31.8799417022096, 31.4202267654863,  8.44624178612283,
      29.8261606993248, 48.6806848614927,  30.7946548609555,  0.256640835551139, 17.1027447983572,
      48.0149553760634, -41.4547015871921, 47.3412174509599,  -12.1087740393677, -36.7578990670318,
      25.1292052009824, 23.9229451353282,  -37.5043476487739, 26.0154536923946,  14.8681946169826,
      14.0563818854373, 41.2019382865639,  4.97727761237993,  12.7202629504647,  -46.2736222458751};
  auto bbb = fitnessScore(aa);
}

MBF6::~MBF6()
{
}

oa::VectorDouble oa::MBF6::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0., n = dv.size();

  auto u = [=](double x, double a, double k, double m) {
    if (x > a) {
      return k * std::pow(x - a, m);
    } else if (x < -a) {
      return k * std::pow(-x - a, m);
    } else {
      return 0.0;
    };
  };

  double A1 = 0, A2 = 0;
  for (size_t i = 0; i < dv.size() - 1; ++i) {
    A1 += std::pow(dv[i] - 1., 2) * (1 + std::pow(std::sin(3 * oa::pi() * dv[i] + 1), 2));
  }

  for (size_t i = 0; i < dv.size(); ++i) {
    A2 += u(dv[i], 5, 100, 4);
  }

  sum = 0.1 * (std::pow(std::sin(3 * oa::pi() * dv[0]), 2) + A1 +
               std::pow(dv[dv.size()-1] - 1, 2) *
                   (1 + std::pow(std::sin(2 * oa::pi() * dv[dv.size() - 1]), 2))) +
        A2;
  return {sum};
}

VectorDouble::size_type oa::MBF6::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::MBF6::bounds()
{
  VectorDouble lb(n_, -50);
  VectorDouble ub(n_, 50);
  return {lb, ub};
}

VectorDouble::size_type oa::MBF6::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type MBF6::correspondenceEstimation()
{
  return 0.000001;
}
