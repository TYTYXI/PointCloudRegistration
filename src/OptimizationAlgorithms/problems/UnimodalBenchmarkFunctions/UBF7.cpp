#include "UBF7.h"

using namespace oa;

oa::VectorDouble oa::UBF7::fitnessScore(const oa::VectorDouble& dv)
{
  double sum = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> r(0.0, 1.0);
  for (size_t i = 0; i < dv.size(); ++i) {
    sum += (i * std::pow(dv[i], 4) );
  }
  return {sum+ r(gen)};
}

VectorDouble::size_type oa::UBF7::dimension()
{
  return n_;
}

std::pair<VectorDouble, VectorDouble> oa::UBF7::bounds()
{
  const VectorDouble lb(n_, -1.28);
  const VectorDouble ub(n_, 1.28);
  return {lb, ub};
}

VectorDouble::size_type oa::UBF7::numOfObjectiveFunction()
{
  return 1;
}

UBF7::UBF7(int n)
    : n_(n)
{
}

UBF7::~UBF7()
{
}

VectorDouble::value_type UBF7::correspondenceEstimation()
{
  return 0.000001;
}
