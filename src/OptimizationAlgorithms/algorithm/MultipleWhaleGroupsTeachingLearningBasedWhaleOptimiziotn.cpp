#include "MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn.h"
#include "TeachingLearningBasedWhaleOptimiziotn.h"

oa::MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn::
    MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn(size_t iteration,
                                                             size_t numOfWhaleGroups,
                                                             size_t interval)
    : iteration_(iteration)
    , numOfWhaleGroups_(numOfWhaleGroups)
    , interval_(interval)
{
}

oa::Population
oa::MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn::optimize(oa::Population pop)
{
  auto& kprob = pop.problem();
  auto dim = kprob.dimension();
  const auto bounds = kprob.bounds();
  const auto& lb = bounds.first;
  const auto& ub = bounds.second;
  auto numOfStudents = pop.size();
  //  std::vector<double> results;

  std::vector<Population> pops(numOfStudents, pop);
  oa::TeachingLearningBasedWhaleOptimiziotn tlbwoa(interval_);

  for (size_t j = 0; j < iteration_; ++j) {
    for (size_t i = 0; i < numOfWhaleGroups_; ++i) {
      //    auto res = tlbo.optimize(pop);
      pops[i] = (tlbwoa.optimize(pops[i]));
      pops[i].updatePoorest();
    }

    if (j != 0 && j % interval_ == 0) {
      for (size_t i = 0; i < numOfWhaleGroups_; ++i) {
        if (i != numOfStudents) {
          pops[i + 1].replaceIndividual(pops[i + 1].poorestDecisionVariablesIndex(),
                                        pops[i].championDecisionVariables());
        } else {
          pops[0].replaceIndividual(pops[0].poorestDecisionVariablesIndex(),
                                    pops[i].championDecisionVariables());
        }
      }
    }
  }

  auto res = std::min_element(
      pops.cbegin(), pops.cend(), [&](const Population& pop1, const Population& pop2) {
        return pop1.championFitnessScores()[0] < pop2.championFitnessScores()[0];
      });
  return *res;
}