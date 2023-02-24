//
// Created by XI on 2022/12/7.
//

// STL Includes
#include <cassert>
#include <stdexcept>
#include <string>

// OA Includes
#include "Population.hpp"
#include "Problems.hpp"

#include <iostream>
#include <tbb/spin_mutex.h>

typedef tbb::spin_mutex FreeListMutexType;
FreeListMutexType FreeListMutex;

using namespace oa;

void Population::initDecisionVariables(Population::PopSizeT popSize)
{
  std::vector<std::pair<VectorDouble, VectorDouble>> tmp(popSize);
  for (PopSizeT i = 0; i < popSize; ++i) {
    tmp[i].first = randomDecisionVector();
    tmp[i].second = problem_.fitnessScore(tmp[i].first);
  }

  for (PopSizeT i = 0; i < popSize; ++i) {
    decisionVariables_.emplace_back(std::move(tmp[i].first));
    fitnessScores_.emplace_back(std::move(tmp[i].second));
    updateChampion(decisionVariables_.back(), fitnessScores_.back());
  }
}

Population::Population()
    : Population(Problem{}, 0u, 0u)
{
}

Population::PopSizeT Population::size() const
{
  return decisionVariables_.size();
}

VectorDouble Population::randomDecisionVector()
{
  VectorDouble out(problem_.dimension());

  const auto kdimension = problem_.dimension();
  const auto& klb = problem_.lowerBoundary();
  const auto& kub = problem_.upperBoundary();

  // Continuous part.
  for (VectorDouble::size_type i = 0; i < kdimension; ++i) {
    std::uniform_real_distribution<double> gen(klb[i], kub[i]);
    out[i] = gen(randomEngine_);
  }

  return out;
}

void Population::updateChampion(VectorDouble dv, VectorDouble fs)
{
  assert(!fs.empty());

  if (championDecision_.empty() || championFitnessScores_[0] > fs[0]) {
    //    std::cout << dv[0] << "  " << dv[1] << std::endl;
    championDecision_ = std::move(dv);
    championFitnessScores_ = std::move(fs);
    //    mut.unlock();
  }
}

Problem& Population::problem()
{
  return problem_;
}

const std::vector<VectorDouble>& Population::decisionVariables() const
{
  return decisionVariables_;
}

const std::vector<VectorDouble>& Population::fitnessScores() const
{
  return fitnessScores_;
}

VectorDouble Population::championDecisionVariables()
{
  return championDecision_;
}

VectorDouble Population::championFitnessScores() const
{
  return championFitnessScores_;
}

void Population::setDvF(Population::PopSizeT i, const VectorDouble& dv, const VectorDouble& f)
{
  FreeListMutexType::scoped_lock lock(FreeListMutex);
  if (i >= size()) {
    throw std::invalid_argument("Trying to access individual at position: " + std::to_string(i) +
                                ", while population has size: " + std::to_string(size()));
  }
  if (f.size() != problem_.numOfObjectiveFunction()) {
    throw std::invalid_argument(
        "Trying to set a fitness of dimension: " + std::to_string(f.size()) +
        ", while the problem's fitness has dimension: " +
        std::to_string(problem_.numOfObjectiveFunction()));
  }

  if (dv.size() != problem_.dimension()) {
    throw std::invalid_argument(
        "Trying to set a decision vector of dimension: " + std::to_string(dv.size()) +
        ", while the problem's dimension is: " + std::to_string(problem_.dimension()));
  }

  decisionVariables_[i].reserve(dv.size());
  fitnessScores_[i].reserve(f.size());

  //    std::cout << dv[0] << "  " << dv[1] << std::endl;
  updateChampion(dv, f);

  decisionVariables_[i].resize(dv.size());
  fitnessScores_[i].resize(f.size());
  std::copy(dv.begin(), dv.end(), decisionVariables_[i].begin());
  std::copy(f.begin(), f.end(), fitnessScores_[i].begin());
  lock.release();
}

void Population::replaceIndividual(Population::PopSizeT index,
                                   const VectorDouble&& newDecisionVariables)
{
  if (newDecisionVariables.size() != problem_.dimension()) {
    return;
  }

  if (index > this->size()) {
    return;
  }

  decisionVariables_[index] = newDecisionVariables;
}

Population::Population(const Population& other)
{
  problem_ = (other.problem_);
  decisionVariables_ = (other.decisionVariables_);
  fitnessScores_ = (other.fitnessScores_);
  championDecision_ = (other.championDecision_);
  championFitnessScores_ = (other.championFitnessScores_);
  randomEngine_ = (other.randomEngine_);
}

Population::Population(Population&& other) noexcept
{
  problem_ = (std::move(other.problem_));
  decisionVariables_ = (std::move(other.decisionVariables_));
  fitnessScores_ = (std::move(other.fitnessScores_));
  championDecision_ = (std::move(other.championDecision_));
  championFitnessScores_ = (std::move(other.championFitnessScores_));
  randomEngine_ = (other.randomEngine_);
}

Population& Population::operator=(const Population& other)
{
  return *this = Population(other);
}

Population& Population::operator=(Population&& other) noexcept
{
  if (this != &other) {
    problem_ = (std::move(other.problem_));
    decisionVariables_ = (std::move(other.decisionVariables_));
    fitnessScores_ = (std::move(other.fitnessScores_));
    championDecision_ = (std::move(other.championDecision_));
    championFitnessScores_ = (std::move(other.championFitnessScores_));
    randomEngine_ = (other.randomEngine_);
  }
  return *this;
}

Population::PopSizeT Population::championDecisionVariablesIndex()
{
  this->updateChampionIndex();
  return championDecisionIndex_;
}

void Population::updateChampionIndex()
{
  auto res = std::find(decisionVariables_.cbegin(), decisionVariables_.cend(), championDecision_);
  championDecisionIndex_ = std::distance(decisionVariables_.cbegin(), res);
}

void Population::updatePoorest(VectorDouble dv, VectorDouble fs)
{
}

void Population::updatePoorestIndex()
{
  auto res =
      std::find(decisionVariables_.cbegin(), decisionVariables_.cend(), poorestDecisionVariables_);
  poorestDecisionIndex_ = std::distance(decisionVariables_.cbegin(), res);
}

void Population::updatePoorest()
{
  auto res = std::min_element(fitnessScores_.cbegin(), fitnessScores_.cend(),
                              [&](const VectorDouble& fitness1, const VectorDouble& fitness2) {
                                return fitness1[0] < fitness2[0];
                              });
  poorestFitnessScores_ = *res;
  auto index = std::distance(fitnessScores_.cbegin(), res);
  poorestDecisionIndex_ = index;
  poorestDecisionVariables_ = decisionVariables_[index];
}

VectorDouble Population::poorestDecisionVariables()
{
  return poorestDecisionVariables_;
}

Population::PopSizeT Population::poorestDecisionVariablesIndex() const
{
  return poorestDecisionIndex_;
}

VectorDouble Population::poorestFitnessScores()
{
  return poorestFitnessScores_;
}
