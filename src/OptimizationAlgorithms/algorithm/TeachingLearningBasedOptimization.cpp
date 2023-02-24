//
// Created by XI on 2022/12/8.
//
#include "TeachingLearningBasedOptimization.h"

// TBB Includes
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>

tbb::spin_mutex mut1;

using namespace oa;

oa::teachingLearningBasedOptimization::teachingLearningBasedOptimization(size_t iteration)
    : iteration_(iteration)
{
}

Population oa::teachingLearningBasedOptimization::optimize(Population pop)
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
  VectorDouble mean(dim, 0);
  VectorDouble teacher(dim);
  std::vector<VectorDouble> newSol(numOfStudents);

  for (auto& sol : newSol) {
    sol.resize(dim);
  }

  // 判断越界
  auto subjectTo = [&](VectorDouble& sol) {
    for (size_t k = 0; k < dim; ++k) {
      if (sol[k] < lb[k] || sol[k] > ub[k]) {
        std::uniform_real_distribution<double> r1(lb[k], ub[k]);
        sol[k] = r1(gen);
      }
    }
  };

  for (size_t i = 0; i < iteration_; ++i) {

    // 计算中值mean和最优解teacher
    for (const auto& kstu : std::as_const(X)) {
      for (size_t j = 0; j < dim; ++j) {
        mean[j] += kstu[j];
      }
    }
    for (size_t j = 0; j < dim; ++j) {
      mean[j] /= (VectorDouble::value_type)numOfStudents;
    }

    teacher = pop.championDecisionVariables();

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, numOfStudents),
        [&](tbb::blocked_range<size_t> range) {
          for (auto j = range.begin(); j != range.end(); ++j) {
            //            std::cout<<j<<std::endl;
            float R;
            float T;
            bool flag = true;
            while (flag) {
              R = r(gen);
              T = (float)t(gen);
              //     std::cout << R << "  "<<T<<"  "<<std::endl;
              newSol[j] = X[j];
              //        float tempVariables[2];
              for (size_t k = 0; k < dim; ++k) {
                newSol[j][k] = newSol[j][k] + R * (teacher[k] - T * mean[k]);
              }

              subjectTo(newSol[j]);

              if (auto res = kprob.fitnessScore(newSol[j]); res[0] < fitnessScores[j][0]) {
                for (int k = 0; k < dim; ++k) {
                  X[j][k] = newSol[j][k];
                }

                //                std::cout<<"ccc"<<X[j][1]<<std::endl;
                fitnessScores[j][0] = res[0];
                pop.setDvF(j, newSol[j], res);
                //                                std::cout << "teaching stage student" << j
                //                                          << " success fitness_score = " <<
                //                                          fitnessScores[j][0] << std::endl;
              } else {
                //                                std::cout << "teaching stage student" << j
                //                                          << " fail fitness_score = " <<
                //                                          fitnessScores[j][0] << std::endl;
              }
              flag = false;
            }
          }
        },
        tbb::auto_partitioner());

    // 计算中值mean和最优解teacher
    std::fill(mean.begin(), mean.end(), 0);

    for (const auto& stu : std::as_const(X)) {
      for (size_t j = 0; j < dim; ++j) {
        mean[j] += stu[j];
      }
    }
    for (size_t j = 0; j < dim; ++j) {
      mean[j] /= (VectorDouble::value_type)numOfStudents;
    }
    teacher = pop.championDecisionVariables();

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, numOfStudents), [&](tbb::blocked_range<size_t> range) {
          for (auto j = range.begin(); j != range.end(); ++j) {
            //            std::cout<<j<<std::endl;
            auto student2 = stu2(gen);

            while (student2 == j) {
              student2 = stu2(gen);
            }
            //            std::cout << "j = " << j << "stu2 = " << student2 << "end" << std::endl;

            bool flag = true;
            float R;

            while (flag) {
              newSol[j] = X[j];
              R = r(gen);
              if (fitnessScores[j][0] < fitnessScores[student2][0]) {
                for (int k = 0; k < dim; ++k) {
                  newSol[j][k] = newSol[j][k] + R * (newSol[j][k] - X[student2][k]);
                }
              } else {
                for (int k = 0; k < dim; ++k) {
                  newSol[j][k] = newSol[j][k] + R * (X[student2][k] - newSol[j][k]);
                }
              }

              subjectTo(newSol[j]);

              if (auto res = kprob.fitnessScore(newSol[j]); res[0] < fitnessScores[j][0]) {
                //                mut1.lock();
                for (int k = 0; k < dim; ++k) {
                  X[j][k] = newSol[j][k];
                }
                fitnessScores[j][0] = res[0];
                //                mut1.unlock();
                pop.setDvF(j, newSol[j], res);
                //                                std::cout << "teaching stage student" << j
                //                                          << " success fitness_score = " <<
                //                                    fitnessScores[j][0] <<
                //                                          std::endl;
              } else {
                //                                std::cout << "teaching stage student" << j
                //                                          << " fail fitness_score = " <<
                //                                          fitnessScores[j][0]
                //                                    <<
                //                                          std::endl;
              }
              flag = false;
            }
          }
        });
  }
  return pop;
}
