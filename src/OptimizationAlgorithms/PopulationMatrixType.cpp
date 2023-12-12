//
// Created by XI on 2023/6/19.
//

#include "PopulationMatrixType.hpp"

oa::PopulationMatrixType::PopulationMatrixType()
    : PopulationMatrixType(Problem{}, 0u, 0u)
{
}

oa::PopulationMatrixType::PopulationMatrixType(const oa::PopulationMatrixType& other)
{
  problem_ = (other.problem_);
  decisionVariables_ = (other.decisionVariables_);
  fitnessScores_ = (other.fitnessScores_);
  championDecision_ = (other.championDecision_);
  championFitnessScores_ = (other.championFitnessScores_);
  randomEngine_ = (other.randomEngine_);
}

oa::PopulationMatrixType::PopulationMatrixType(oa::PopulationMatrixType&& other) noexcept
{
  problem_ = (std::move(other.problem_));
  decisionVariables_ = (std::move(other.decisionVariables_));
  fitnessScores_ = (std::move(other.fitnessScores_));
  championDecision_ = (std::move(other.championDecision_));
  championFitnessScores_ = (std::move(other.championFitnessScores_));
  randomEngine_ = (other.randomEngine_);
}

oa::PopulationMatrixType& oa::PopulationMatrixType::operator=(const oa::PopulationMatrixType& other)
{
  return *this = PopulationMatrixType(other);
}

oa::PopulationMatrixType&
oa::PopulationMatrixType::operator=(oa::PopulationMatrixType&& other) noexcept
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

void oa::PopulationMatrixType::initDecisionVariables(oa::PopulationMatrixType::PopSizeT popSize)
{
  randomDecisionVector();
  return;
//    std::vector<std::pair<VectorDouble, VectorDouble>> tmp(popSize);
//    for (PopSizeT i = 0; i < popSize; ++i) {
//      tmp[i].first = randomDecisionVector();
//      tmp[i].second = problem_.fitnessScore(tmp[i].first);
//    }
//
//    for (PopSizeT i = 0; i < popSize; ++i) {
//      decisionVariables_.emplace_back(std::move(tmp[i].first));
//      fitnessScores_.emplace_back(std::move(tmp[i].second));
//      updateChampion(decisionVariables_.back(), fitnessScores_.back());
//    }
}

oa::Problem& oa::PopulationMatrixType::problem()
{
  return problem_;
}

oa::PopulationMatrixType::PopSizeT oa::PopulationMatrixType::size() const
{
  return 0;
}

void oa::PopulationMatrixType::setDvF(oa::PopulationMatrixType::PopSizeT, const Eigen::VectorXd&,
                                      const Eigen::VectorXd&)
{
}

Eigen::VectorXd oa::PopulationMatrixType::randomDecisionVector()
{
  const auto kdimension = problem_.dimension();
  const auto& klb = problem_.lowerBoundary();
  const auto& kub = problem_.upperBoundary();

  std::vector<std::uniform_real_distribution<double>> gens;

  // Continuous part.
  for (VectorDouble::size_type i = 0; i < kdimension; ++i) {
    gens.emplace_back(klb[i], kub[i]);
  }

  Eigen::VectorXd out = Eigen::VectorXd::Zero(kdimension).unaryExpr([&](double dummy) {
    return gens[0](randomEngine_);
  });

  return out;
}

Eigen::VectorXd oa::PopulationMatrixType::championDecisionVariables()
{
  return Eigen::VectorXd();
}

oa::PopulationMatrixType::PopSizeT oa::PopulationMatrixType::championDecisionVariablesIndex()
{
  return 0;
}

Eigen::VectorXd oa::PopulationMatrixType::championFitnessScores() const
{
  return Eigen::VectorXd();
}

Eigen::VectorXd oa::PopulationMatrixType::poorestDecisionVariables()
{
  return Eigen::VectorXd();
}

oa::PopulationMatrixType::PopSizeT oa::PopulationMatrixType::poorestDecisionVariablesIndex() const
{
  return 0;
}

Eigen::VectorXd oa::PopulationMatrixType::poorestFitnessScores()
{
  return Eigen::VectorXd();
}

const Eigen::MatrixXd& oa::PopulationMatrixType::decisionVariables() const
{
  return decisionVariables_;
}

const Eigen::MatrixXd& oa::PopulationMatrixType::fitnessScores() const
{
  return fitnessScores_;
}

void oa::PopulationMatrixType::replaceIndividual(oa::PopulationMatrixType::PopSizeT index,
                                                 const Eigen::VectorXd&& newDecisionVariables)
{
}

void oa::PopulationMatrixType::updatePoorest()
{
}

void oa::PopulationMatrixType::updateChampion(Eigen::VectorXd dv, Eigen::VectorXd fs)
{
}

void oa::PopulationMatrixType::updateChampionIndex()
{
}

void oa::PopulationMatrixType::updatePoorest(Eigen::VectorXd dv, Eigen::VectorXd fs)
{
}

void oa::PopulationMatrixType::updatePoorestIndex()
{
}
