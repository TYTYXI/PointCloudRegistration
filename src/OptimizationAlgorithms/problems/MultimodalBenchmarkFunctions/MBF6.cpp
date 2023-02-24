#include "MBF6.h"

using namespace oa;

MBF6::MBF6(int n):n_(n)
{
}

MBF6::~MBF6()
{
}

oa::VectorDouble oa::MBF6::fitnessScore(const oa::VectorDouble& dv)
{
  return oa::VectorDouble();
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
