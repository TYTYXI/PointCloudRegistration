#include "MBF5.h"
#include "Constants.hpp"

using namespace oa;

MBF5::MBF5(int n)
    : n_(n)
{
}

MBF5::~MBF5()
{
}

oa::VectorDouble oa::MBF5::fitnessScore(const oa::VectorDouble& dv)
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

  auto y = [=](double x) { return 1. + (x + 1.0) / 4.0; };

  double A1 = 0, A2 = 0;
  for (size_t i = 0; i < dv.size() - 1; ++i) {
    A1 += std::pow(y(dv[i]) - 1., 2) * (1 + 10 * std::pow(std::sin(oa::pi() * y(dv[i + 1])), 2));
  }

  for (size_t i = 0; i < dv.size(); ++i) {
    A2 += u(dv[i], 10, 100, 4);
  }
  auto aa = 10 * std::sin(oa::pi() * y(dv[0]));
  sum = (oa::pi() / n) * (10 * std::pow(std::sin(oa::pi() * y(dv[0])), 2) + A1 +
                          std::pow(y(dv.back()) - 1., 2)) +
        A2;
  return {sum};
}

VectorDouble::size_type oa::MBF5::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::MBF5::bounds()
{
  VectorDouble lb(n_, -50);
  VectorDouble ub(n_, 50);
  return {lb, ub};
}

VectorDouble::size_type oa::MBF5::numOfObjectiveFunction()
{
  return 1;
}

VectorDouble::value_type MBF5::correspondenceEstimation()
{
  return 0.000001;
}
