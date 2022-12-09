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

  VectorDouble championDecisionVariables();

  VectorDouble championFitnessScores();

  const std::vector<VectorDouble>& decisionVariables() const;

  const std::vector<VectorDouble>& fitnessScores() const;

  void updateChampion(VectorDouble dv, VectorDouble fs);

private:
  // 要优化的问题.
  Problem problem_;
  // 决策变量
  std::vector<VectorDouble> decisionVariables_;
  // 目标函数值
  std::vector<VectorDouble> fitnessScores_;
  // The Champion chromosome
  VectorDouble championDecision_;
  // The Champion fitness
  VectorDouble championFitnessScores_;
  // Random engine.
  mutable RandomEngineType randomEngine_;
  // 随机种子
  int seed_;
  //
};
} // namespace oa

#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_POPULATION_HPP_
