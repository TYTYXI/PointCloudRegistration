//
// Created by XI on 2022/12/9.
//

#include "MultipleClassTeachingLearningBasedOptimization.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include <tbb/parallel_for.h>

using namespace oa;

multipleClassTeachingLearningBasedOptimization::multipleClassTeachingLearningBasedOptimization(
    size_t iteration, size_t numOfClasses)
    : iteration_(iteration)
    , numOfClasses_(numOfClasses)
{
}

Population multipleClassTeachingLearningBasedOptimization::optimize(Population pop)
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
  std::vector<VectorDouble> newSol(numOfStudents);
  Eigen::Tensor<float, 3> a((long long)numOfClasses_, (long long)numOfStudents, (long long)dim);

  for (auto& sol : newSol) {
    sol.resize(dim);
  }

  // 判断越界
  auto subjectTo = [&](VectorDouble& sol) {
    for (size_t k = 0; k < dim; ++k) {
      if (sol[k] < lb[k] || sol[k] > ub[k]) {
        std::uniform_real_distribution<double> r1(lb[k], ub[k]);
        sol[k] = r1(gen);
        return true;
      }
    }
    return true;
  };
  return pop;
}