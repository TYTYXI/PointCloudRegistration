//
// Created by XI on 2022/12/6.
//
#include <Eigen/Eigen>

#include "benchmark/benchmark.h"

#include "../algorithm/ArtificialBeeColony.h"
// #include "MultipleClassTeachingLearningBasedOptimization.h"
#include "../algorithm/TeachingLearningBasedOptimization.h"
#include "../problems/test1Problem.h"
BENCHMARK_MAIN();

void BM_TLBO(benchmark::State& state)
{
  for (auto _ : state) {
  oa::Problem prob{oa::test1Problem()};
  oa::Population pop{prob,30};

  oa::teachingLearningBasedOptimization tlbo(20);

  auto res = tlbo.optimize(pop);

//        std::cout << res.championDecisionVariables()[0] << "  " <<
//        res.championDecisionVariables()[1]
//                  << "   " << res.championFitnessScores()[0] << std::endl;
  }
}

void BM_MCTLBO(benchmark::State& state)
{
  for (auto _ : state) {
  }
}

BENCHMARK(BM_TLBO)->Unit(benchmark::kMillisecond)->Iterations(1000)->Repetitions(100);
BENCHMARK(BM_MCTLBO)->Unit(benchmark::kMillisecond)->Iterations(1000);
