//
// Created by XI on 2023/6/19.
//

// TBB Includes
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>

#include <Eigen/Eigen>

#include "PopulationMatrixType.hpp"
#include "TeachingLearningBasedOptimizationMatrixType.h"

oa::TeachingLearningBasedOptimizationMatrixType::TeachingLearningBasedOptimizationMatrixType(
    size_t iteration)
    : iteration_(iteration)
{
}

oa::PopulationMatrixType
oa::TeachingLearningBasedOptimizationMatrixType::optimize(oa::PopulationMatrixType pop)
{
  auto& kprob = pop.problem();
  auto dim = kprob.dimension();
  const auto bounds = kprob.bounds();
  const auto& lb = bounds.first;
  const auto& ub = bounds.second;
  auto numOfStudents = pop.size();

  // TLBO参数准备
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> r(0.0, 1.0);
  std::uniform_int_distribution<int> t(1, 2);
  std::uniform_int_distribution<int> stu2(0, numOfStudents - 1);
  auto X = pop.decisionVariables();
  auto fitnessScores = pop.fitnessScores();

  Eigen::VectorXd mean(dim);
  mean.setZero();
  Eigen::VectorXd teacher(dim);
  Eigen::MatrixXd newSol(dim, numOfStudents);

  // 判断越界
  auto subjectTo = [&](Eigen::VectorXd& sol) {
    for (size_t k = 0; k < dim; ++k) {
      if (sol[k] < lb[k] || sol[k] > ub[k]) {
        std::uniform_real_distribution<double> r1(lb[k], ub[k]);
        sol[k] = r1(gen);
      }
    }
  };
  auto begin = std::chrono::high_resolution_clock::now();

  for (size_t i = 0; i < iteration_; ++i) {

    // 计算中值mean和最优解teacher
    for (size_t j = 0; j < numOfStudents; ++j) {
      mean += X.block(0, j, dim, 1);
    }
    mean /= numOfStudents;
  }

  pop.championFitnessScores();

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
  return pop;
}
