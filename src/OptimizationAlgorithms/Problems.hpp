//
// Created by XI on 2022/12/7.
//

#ifndef POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_HPP_
#define POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_HPP_

#include "OptimizationAlgorithmsGlobal.h"
#include "Types.h"
#include <memory>

namespace oa {

struct ALGORITHM_EXPORT ProbInnerBase
{
public:
  virtual std::unique_ptr<ProbInnerBase> clone() const = 0;

  virtual VectorDouble fitnessScore(const VectorDouble& dv) = 0;

  virtual std::pair<VectorDouble, VectorDouble> bounds() = 0;

  virtual VectorDouble::size_type dimension() = 0;

  virtual VectorDouble::size_type numOfObjectiveFunction() = 0;

  virtual VectorDouble::value_type correspondenceEstimation() = 0;
};

template <class T>
struct ALGORITHM_EXPORT ProbInner : public ProbInnerBase
{
public:
  explicit ProbInner(const T& value)
      : value(value)
  {
  }

  explicit ProbInner(T&& value)
      : value(std::move(value))
  {
  }

  std::unique_ptr<ProbInnerBase> clone() const final
  {
    return std::make_unique<ProbInner>(value);
  }

  VectorDouble fitnessScore(const VectorDouble& dv) final
  {
    return value.fitnessScore(dv);
  };

  std::pair<VectorDouble, VectorDouble> bounds() final
  {
    return value.bounds();
  };

  VectorDouble::size_type dimension() final
  {
    return value.dimension();
  };

  VectorDouble::size_type numOfObjectiveFunction() final
  {
    return value.numOfObjectiveFunction();
    //          return 1;
  };

  VectorDouble::value_type correspondenceEstimation() final
  {
    return value.correspondenceEstimation();
    //          return 1;
  };

public:
  T value;
};

class ALGORITHM_EXPORT Problem
{
public:
  Problem();

  template <class T>
  explicit Problem(T&& probBase)
      : probPtr_(std::make_unique<ProbInner<std::remove_cv_t<std::remove_reference_t<T>>>>(
            std::forward<T>(probBase)))
  {
    initProblemBase();
  }

  // Copy constructor.
  Problem(const Problem&);
  // Move constructor.
  Problem(Problem&&) noexcept;
  // Move assignment operator
  Problem& operator=(Problem&&) noexcept;
  // Copy assignment operator
  Problem& operator=(const Problem&);

  VectorDouble fitnessScore(const VectorDouble& dv);

  VectorDouble::size_type dimension();
  std::pair<VectorDouble, VectorDouble> bounds();

  VectorDouble::size_type numOfObjectiveFunction();

  VectorDouble::value_type correspondenceEstimation();

  const VectorDouble& lowerBoundary();

  const VectorDouble& upperBoundary();

private:
  void initProblemBase();

private:
  std::unique_ptr<ProbInnerBase> probPtr_;
  VectorDouble lowerBoundary_;
  VectorDouble upperBoundary_;
  VectorDouble::size_type numOfObjectiveFunction_;
  VectorDouble::value_type correspondenceEstimation_;
};
} // namespace oa
#endif // POINTCLOUDREGISTRATION_SRC_OPTIMIZATIONALGORITHMS_PROBLEMS_HPP_
