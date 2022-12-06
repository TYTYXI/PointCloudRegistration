//
// Created by XI on 2022/12/6.
//
#include <Eigen/Eigen>

#include "benchmark/benchmark.h"

#include "ArtificialBeeColony.h"
#include "MultipleClassTeachingLearningBasedOptimization.h"
#include "TeachingLearningBasedOptimization.h"

template <class T>
struct TargetFunc
{
  typedef T ParamsType;
  typedef T ResultType;

  static constexpr int const numOfParams = 3;

  ResultType operator()(ResultType* a) const
  {
    auto result = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    return result;
  }
};

BENCHMARK_MAIN();

void BM_TLBO(benchmark::State& state)
{
  for (auto _ : state) {
    float b[6];
    for (size_t i = 0; i < 3; ++i) {
      b[2 * i] = -1.;
      b[2 * i + 1] = 1.;
    }
    ioa::teachingLearningBasedOptimization<TargetFunc<float>> tlbo(20, 20);
    tlbo.setLimits(b);
    tlbo.initTLBO();
    tlbo.optimize();
    float params[3];
    tlbo.parameters(params);
    //    delete[] b;
    //  std::cout << params[0] << "  " << params[1] << "  " << params[2] << std::endl;
  }
}

void BM_MCTLBO(benchmark::State& state)
{
  for (auto _ : state) {
  auto b = new float[6];
  for (size_t i = 0; i < 3; ++i) {
    b[2 * i] = -1.;
    b[2 * i + 1] = 1.;
  }
  ioa::multipleClassTeachingLearningBasedOptimization<TargetFunc<float>> mctlbo(20, 6, 20);
  mctlbo.setLimits(b);
  mctlbo.optimize();
  float params[3];
  mctlbo.parameters(params);
//  std::cout << params[0] << "  " << params[1] << "  " << params[2] << std::endl;
  }
}

BENCHMARK(BM_TLBO)->Unit(benchmark::kMillisecond)->Iterations(1000);
BENCHMARK(BM_MCTLBO)->Unit(benchmark::kMillisecond)->Iterations(1000);
