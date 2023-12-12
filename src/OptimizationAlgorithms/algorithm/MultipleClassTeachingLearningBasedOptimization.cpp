//
// Created by XI on 2022/12/9.
//
#include <limits>

#include "MultipleClassTeachingLearningBasedOptimization.h"
#include "SmallGroupTeachingLearningBasedOptimization.h"
#include "TeachingLearningBasedOptimization.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include <tbb/parallel_for.h>

using namespace oa;

multipleClassTeachingLearningBasedOptimization::multipleClassTeachingLearningBasedOptimization(
    size_t iteration, size_t numOfClasses, size_t interval)
    : iteration_(iteration)
    , numOfClasses_(numOfClasses)
    , interval_(interval)
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
  //  std::vector<double> results;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> r(0.0, 1.0);

  std::vector<Population> pops(numOfStudents, pop);
  oa::smallGroupTeachingLearningBasedOptimization tlbo(interval_);

  for (size_t j = 0; j < iteration_; ++j) {
    for (size_t i = 0; i < numOfClasses_; ++i) {
      //    auto res = tlbo.optimize(pop);
      pops[i] = (tlbo.optimize(pops[i]));
      pops[i].updatePoorest();
    }

//    if (j == 0) {
//      lastFitnessScore_ = std::numeric_limits<double>::max();
//    } else {
//      auto res = std::min_element(
//          pops.cbegin(), pops.cend(), [&](const Population& pop1, const Population& pop2) {
//            return pop1.championFitnessScores()[0] < pop2.championFitnessScores()[0];
//          });
//      if (std::abs(res->championFitnessScores()[0]-lastFitnessScore_) < 0.000000001) {
//        break;
//      }
//    }

    if (j != 0 && j % interval_ == 0) {
      for (size_t i = 0; i < numOfClasses_; ++i) {

        auto c = pops[i].championDecisionVariables();
        auto max_c = std::max_element(c.begin(), c.end());
        auto min_c = std::min_element(c.begin(), c.end());

        VectorDouble newc(c.size());
        for (size_t k = 0; k < c.size(); ++k) {
          newc[k] = lb[k] + ub[k] - c[k];
          //          newc[k] = r(gen) * (*max_c + *min_c) - c[k];
        }
        if (kprob.fitnessScore(newc) > kprob.fitnessScore(c)) {
          newc = c;
        }
        if (i != numOfStudents) {
          //          pops[i + 1].replaceIndividual(pops[i + 1].poorestDecisionVariablesIndex(),
          //                                        pops[i].championDecisionVariables());
          pops[i + 1].replaceIndividual(pops[i + 1].poorestDecisionVariablesIndex(),
                                        std::move(newc));
        } else {
          //          pops[0].replaceIndividual(pops[0].poorestDecisionVariablesIndex(),
          //                                    pops[i].championDecisionVariables());
          pops[0].replaceIndividual(pops[0].poorestDecisionVariablesIndex(), std::move(newc));
        }
      }
    }

    //    auto res = std::min_element(
    //        pops.cbegin(), pops.cend(), [&](const Population& pop1, const Population& pop2) {
    //          return pop1.championFitnessScores()[0] < pop2.championFitnessScores()[0];
    //        });
    //
    //    results.emplace_back(res->championFitnessScores()[0]);
    //    std::cout << results.back() << std::endl;
    //    if (results.size() < 2) {
    //      continue;
    //    }
    //
    ////    if (std::abs(results[j] - results[j - 1] )< kprob.correspondenceEstimation()) {
    ////      break;
    ////    }
  }

  auto res = std::min_element(
      pops.cbegin(), pops.cend(), [&](const Population& pop1, const Population& pop2) {
        return pop1.championFitnessScores()[0] < pop2.championFitnessScores()[0];
      });
  return *res;
}