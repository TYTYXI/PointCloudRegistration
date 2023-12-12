//
// Created by XI on 2023/6/19.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_POPULATIONMATRIXTYPE_HPP_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_POPULATIONMATRIXTYPE_HPP_

// OA Includes
#include "OptimizationAlgorithmsGlobal.h"
#include "Problems.hpp"
#include "Types.h"

#include <Eigen/Core>

namespace oa {
class ALGORITHM_EXPORT PopulationMatrixType
{
public:
  using PopSizeT = unsigned long long;

  PopulationMatrixType();

  // Copy constructor.
  PopulationMatrixType(const PopulationMatrixType&);
  // Move constructor.
  PopulationMatrixType(PopulationMatrixType&&) noexcept;
  // Copy assignment operator.
  PopulationMatrixType& operator=(const PopulationMatrixType&);
  // Move assignment operator.
  PopulationMatrixType& operator=(PopulationMatrixType&&) noexcept;

  void initDecisionVariables(PopSizeT size);

  template <typename T>
  explicit PopulationMatrixType(T&& prob, PopSizeT popSize = 0, int seed = 0)
      : problem_(std::forward<T>(prob))
      , randomEngine_(seed)
      , seed_(seed)
  {
    initDecisionVariables(popSize);
  }

  Problem& problem();

  PopSizeT size() const;

  // 设置新的决策变量值和适度值
  void setDvF(PopSizeT, const Eigen::VectorXd&, const Eigen::VectorXd&);

  // 生成随机的决策变量
  Eigen::VectorXd randomDecisionVector();
  // 最优解的决策变量
  Eigen::VectorXd championDecisionVariables();
  PopSizeT championDecisionVariablesIndex();
  // 最优解值
  Eigen::VectorXd championFitnessScores() const;
  // 最差解的决策变量
  Eigen::VectorXd poorestDecisionVariables();
  PopSizeT poorestDecisionVariablesIndex() const;
  // 最差解值
  Eigen::VectorXd poorestFitnessScores();

  // 返回种群的决策变量
  const Eigen::MatrixXd& decisionVariables() const;

  const Eigen::MatrixXd& fitnessScores() const;

  void replaceIndividual(PopulationMatrixType::PopSizeT index,
                         const Eigen::VectorXd&& newDecisionVariables);

  void updatePoorest();

private:
  void updateChampion(Eigen::VectorXd dv, Eigen::VectorXd fs);
  void updateChampionIndex();
  void updatePoorest(Eigen::VectorXd dv, Eigen::VectorXd fs);
  void updatePoorestIndex();

private:
  // 要优化的问题.
  Problem problem_;
  // 决策变量
  Eigen::MatrixXd decisionVariables_;
  // 目标函数值
  Eigen::MatrixXd fitnessScores_;
  // The Champion dv
  Eigen::VectorXd championDecision_;
  PopSizeT championDecisionIndex_;
  // The Champion fitness
  Eigen::VectorXd championFitnessScores_;
  // The Poorest dv
  Eigen::VectorXd poorestDecisionVariables_;
  PopSizeT poorestDecisionIndex_;
  // The Poorest fitness
  Eigen::VectorXd poorestFitnessScores_;
  // Random engine.
  mutable RandomEngineType randomEngine_;
  // 随机种子
  int seed_;
  //
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_POPULATIONMATRIXTYPE_HPP_
