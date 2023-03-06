#include "TeachingLearningBasedWhaleOptimiziotn.h"
#include "Constants.hpp"

// TBB Includes
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>

oa::TeachingLearningBasedWhaleOptimiziotn::TeachingLearningBasedWhaleOptimiziotn(size_t iteration)
    : iteration_(iteration)
{
}

oa::Population oa::TeachingLearningBasedWhaleOptimiziotn::optimize(oa::Population pop)
{
  auto& kprob = pop.problem();
  auto dim = kprob.dimension();
  const auto bounds = kprob.bounds();
  const auto& lb = bounds.first;
  const auto& ub = bounds.second;
  auto numOfWhales = pop.size();

  // WOA参数准备
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> t(1, 2);
  std::uniform_real_distribution<float> p(0.0, 1.0);
  std::uniform_real_distribution<float> r1(0.0, 1.0);
  std::uniform_real_distribution<float> r2(0.0, 1.0);
  std::uniform_real_distribution<float> l(-1.0, 1.0);
  std::uniform_int_distribution<int> whale2(0, numOfWhales - 1);
  double alpha_div = 2.0 / iteration_;
  double alpha = alpha_div * 1;
  auto X = pop.decisionVariables();
  auto fitnessScores = pop.fitnessScores();
  VectorDouble mean(dim, 0);
  VectorDouble leaderWhale(dim);
  std::vector<VectorDouble> newSol(numOfWhales);
  const double b = 1.0;

  for (auto& sol : newSol) {
    sol.resize(dim);
  }

  // 判断越界
  auto subjectTo = [&](VectorDouble& sol) {
    for (size_t k = 0; k < dim; ++k) {
      if (sol[k] < lb[k] || sol[k] > ub[k]) {
//        std::uniform_real_distribution<double> r(lb[k], ub[k]);
//        sol[k] = r(gen);
              if (sol[k] < lb[k]) {
                sol[k] = lb[k];
              } else if (sol[k] > ub[k]) {
                sol[k] = ub[k];
              }
      }
    }
  };

  for (size_t i = 0; i < iteration_; ++i) {
    // 包围猎物
    for (const auto& kstu : std::as_const(X)) {
      for (size_t j = 0; j < dim; ++j) {
        mean[j] += kstu[j];
      }
    }
    for (size_t j = 0; j < dim; ++j) {
      mean[j] /= (VectorDouble::value_type)numOfWhales;
    }

    leaderWhale = pop.championDecisionVariables();
    auto cahm = pop.championFitnessScores()[0];
    //    std::cout << cahm << std::endl;
    alpha = 2.0 - alpha_div * ((double)i);
        tbb::parallel_for(tbb::blocked_range<size_t>(0, numOfWhales),
                          [&](tbb::blocked_range<size_t> range) {
                            for (auto j = range.begin(); j != range.end(); ++j) {
//    for (auto j = 0; j != X.size(); ++j) {

      auto A1 = 2 * alpha * r1(gen) - alpha;
      auto C1 = 2 * r2(gen);
      float T = t(gen);

      newSol[j] = X[j];
      // 发泡网攻击
      if (p(gen) < 0.5) {

        // 收缩包围
        if (std::abs(A1) < 1) {
          // 包围猎物
          for (size_t k = 0; k < dim; ++k) {
//                        auto Dk = std::abs(C1 * leaderWhale[k] - newSol[j][k]);
            newSol[j][k] = newSol[j][k] + A1 * (C1 * leaderWhale[k] - T * mean[k]);
//            newSol[j][k] = leaderWhale[k] - A1 * Dk;
          }
        } else {
          // 搜索捕食
           auto rand_whale = whale2(gen);
            while (rand_whale == j) {
              rand_whale = whale2(gen);
            }
          for (size_t k = 0; k < dim; ++k) {

            if (fitnessScores[j][0] < fitnessScores[rand_whale][0]) {
                newSol[j][k] = newSol[j][k] + A1 * (newSol[j][k] - C1*X[rand_whale][k]);
            } else {
                newSol[j][k] = newSol[j][k] + A1 * (C1*X[rand_whale][k] - newSol[j][k]);
            }
//
//            auto D_rand = std::abs(C1 * newSol[rand_whale][k] - newSol[j][k]);
//            newSol[j][k] = newSol[rand_whale][k] - A1 * D_rand;
          }
        }
      } else {
        auto rand_l = l(gen);
        for (size_t k = 0; k < dim; ++k) {
          auto Dk = std::abs(leaderWhale[k] - newSol[j][k]);
          newSol[j][k] =
              leaderWhale[k] + Dk * std::pow(oa::e(), rand_l * b) * std::cos(2 * oa::pi() * rand_l);
        }
      }

      subjectTo(newSol[j]);

      for (int k = 0; k < dim; ++k) {
        X[j][k] = newSol[j][k];
      }
      auto res = kprob.fitnessScore(newSol[j]);
      //      std::cout << i << "   " << res[0] << std::endl;
      fitnessScores[j][0] = res[0];
      //                mut1.unlock();
      pop.setDvF(j, newSol[j], res);
    }
                          });
  }
  return pop;
}
