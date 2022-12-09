//
// Created by XI on 2022/12/6.
//
#include <Eigen/Eigen>

#include "benchmark/benchmark.h"

#include "../algorithm/ArtificialBeeColony.h"
// #include "MultipleClassTeachingLearningBasedOptimization.h"
#include "../algorithm/TeachingLearningBasedOptimization.h"
#include "../problems/test1Problem.h"
#include "problems/test2Problem.h"
BENCHMARK_MAIN();

void BM_TLBO1(benchmark::State& state)
{
  for (auto _ : state) {
    oa::Problem prob{oa::test1Problem()};
    oa::Population pop{prob, 30};

    oa::teachingLearningBasedOptimization tlbo(20);

    auto res = tlbo.optimize(pop);

    //        std::cout << res.championDecisionVariables()[0] << "  " <<
    //        res.championDecisionVariables()[1]
    //                  << "   " << res.championFitnessScores()[0] << std::endl;
  }
}

void BM_TLBO2(benchmark::State& state)
{
  for (auto _ : state) {
    oa::Problem prob{oa::test2Problem()};
    oa::Population pop{prob, 35};

    oa::teachingLearningBasedOptimization tlbo(25);

    auto res = tlbo.optimize(pop);
  }
}

void BM_MCTLBO(benchmark::State& state)
{
  for (auto _ : state) {
  }
}

BENCHMARK(BM_TLBO1)->Unit(benchmark::kMillisecond)->Iterations(1000)->Repetitions(20);
BENCHMARK(BM_TLBO2)->Unit(benchmark::kMillisecond)->Iterations(1000)->Repetitions(20);
BENCHMARK(BM_MCTLBO)->Unit(benchmark::kMillisecond)->Iterations(1000);
