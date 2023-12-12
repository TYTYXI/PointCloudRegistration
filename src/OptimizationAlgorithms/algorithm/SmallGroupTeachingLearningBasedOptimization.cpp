//
// Created by XI on 2023/10/16.
//

#include "SmallGroupTeachingLearningBasedOptimization.h"

// TBB Includes
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>
#include <tbb/spin_rw_mutex.h>

using namespace oa;

oa::smallGroupTeachingLearningBasedOptimization::smallGroupTeachingLearningBasedOptimization(
    size_t iteration)
    : iteration_(iteration)
{
}

Population oa::smallGroupTeachingLearningBasedOptimization::optimize(Population pop)
{
  auto& kprob = pop.problem();
  auto dim = kprob.dimension();
  const auto bounds = kprob.bounds();
  const auto& lb = bounds.first;
  const auto& ub = bounds.second;
  auto numOfStudents = pop.size();

  tbb::spin_mutex mut1;

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

  // Tent 混沌序列
  std::vector<double> tentMapping;
  int count = 0;
  tentMapping.emplace_back(r(gen));
  double bb = 0.499;
  auto calculate_tent_mapping = [&](int index) {
    auto zk = tentMapping[index - 1];
    if (0 < zk && zk <= bb) {
      return zk / bb;
    } else if (bb < zk && zk <= 1) {
      return (1. - zk) / (1. - bb);
    }
  };

  tbb::spin_rw_mutex mtx;
  auto rx2 = [&]() {
    mtx.lock();
    count++;
    tentMapping.emplace_back(calculate_tent_mapping(count));
    auto res = 8 * (tentMapping[count] * (1 - tentMapping[count]));
    mtx.unlock();
    return res;
  };

  auto rx1 = [&]() {
    mtx.lock();
    count++;
    tentMapping.emplace_back(calculate_tent_mapping(count));
    auto res = 4 * (tentMapping[count] * (1 - tentMapping[count]));
    mtx.unlock();
    return res;
  };

  // 判断越界
  auto subjectTo = [&](VectorDouble& sol) {
    for (size_t k = 0; k < dim; ++k) {
      if (sol[k] < lb[k] || sol[k] > ub[k]) {
        std::uniform_real_distribution<double> r1(lb[k], ub[k]);
        sol[k] = r1(gen);
      }
    }
  };

  auto selection_sort = [&](std::vector<VectorDouble>& arr, std::vector<VectorDouble>& fitScores) {
    for (int i = 0; i < arr.size() - 1; i++) {
      int min = i;
      for (int j = i + 1; j < arr.size(); j++)
        if (fitnessScores[j][0] < fitnessScores[min][0])
          min = j;
      std::swap(arr[i], arr[min]);
      std::swap(fitnessScores[i], fitnessScores[min]);
    }
  };

  int numOfGroup = numOfStudents / 5;
  std::vector<VectorDouble> groupMeans(numOfGroup,VectorDouble(dim));

  for (size_t i = 0; i < iteration_; ++i) {

    selection_sort(X, fitnessScores);
    // 计算中值mean和最优解teacher
    int meanIndex = 0;
    for (const auto& kstu : std::as_const(X)) {
      for (size_t j = 0; j < dim; ++j) {
        groupMeans[meanIndex / 5][j] += kstu[j];
        //        mean[j] += kstu[j];
      }
      meanIndex++;
    }
    for (auto& kstu : groupMeans) {
      for (size_t j = 0; j < dim; ++j) {
        kstu[j] /= 5;
        //        mean[j] += kstu[j];
      }
    }
    //    for (size_t j = 0; j < dim; ++j) {
    //      mean[j] /= (VectorDouble::value_type)numOfStudents;
    //      mean[j] /= (VectorDouble::value_type)numOfStudents;
    //    }

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
                newSol[j][k] = newSol[j][k] + rx1() * (teacher[k] - T * groupMeans[j/5][k]);
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
                  newSol[j][k] = newSol[j][k] + rx1() * (newSol[j][k] - X[student2][k]);
                }
              } else {
                for (int k = 0; k < dim; ++k) {
                  newSol[j][k] = newSol[j][k] + rx1() * (X[student2][k] - newSol[j][k]);
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
