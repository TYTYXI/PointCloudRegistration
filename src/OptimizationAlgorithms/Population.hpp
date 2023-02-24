//
// Created by XI on 2022/12/7.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_POPULATION_HPP_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_POPULATION_HPP_

// OA Includes
#include "OptimizationAlgorithmsGlobal.h"
#include "Problems.hpp"
#include "Types.h"

// STL includes
#include <vector>

namespace oa {

class ALGORITHM_EXPORT Population
{
public:
  typedef std::vector<VectorDouble>::size_type PopSizeT;

  Population();

  // Copy constructor.
  Population(const Population&);
  // Move constructor.
  Population(Population&&) noexcept;
  // Copy assignment operator.
  Population& operator=(const Population&);
  // Move assignment operator.
  Population& operator=(Population&&) noexcept;

  void initDecisionVariables(PopSizeT size);

  template <typename T>
  explicit Population(T&& prob, PopSizeT popSize = 0, int seed = 0)
      : problem_(std::forward<T>(prob))
      , randomEngine_(seed)
      , seed_(seed)
  {
    initDecisionVariables(popSize);
  }

  Problem& problem();

  PopSizeT size() const;

  // 设置新的决策变量值和适度值
  void setDvF(PopSizeT, const VectorDouble&, const VectorDouble&);

  // 生成随机的决策变量
  VectorDouble randomDecisionVector();
  // 最优解的决策变量
  VectorDouble championDecisionVariables();
  PopSizeT championDecisionVariablesIndex();
  // 最优解值
  VectorDouble championFitnessScores() const;
  // 最差解的决策变量
  VectorDouble poorestDecisionVariables();
  PopSizeT poorestDecisionVariablesIndex() const;
  // 最差解值
  VectorDouble poorestFitnessScores();

  // 返回种群的决策变量
  const std::vector<VectorDouble>& decisionVariables() const;

  const std::vector<VectorDouble>& fitnessScores() const;

  void replaceIndividual(Population::PopSizeT index, const VectorDouble&& newDecisionVariables);

  void updatePoorest();

private:
  void updateChampion(VectorDouble dv, VectorDouble fs);
  void updateChampionIndex();
  void updatePoorest(VectorDouble dv, VectorDouble fs);
  void updatePoorestIndex();

private:
  // 要优化的问题.
  Problem problem_;
  // 决策变量
  std::vector<VectorDouble> decisionVariables_;
  // 目标函数值
  std::vector<VectorDouble> fitnessScores_;
  // The Champion dv
  VectorDouble championDecision_;
  PopSizeT championDecisionIndex_;
  // The Champion fitness
  VectorDouble championFitnessScores_;
  // The Poorest dv
  VectorDouble poorestDecisionVariables_;
  PopSizeT poorestDecisionIndex_;
  // The Poorest fitness
  VectorDouble poorestFitnessScores_;
  // Random engine.
  mutable RandomEngineType randomEngine_;
  // 随机种子
  int seed_;
  //
};
} // namespace oa

#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_POPULATION_HPP_
