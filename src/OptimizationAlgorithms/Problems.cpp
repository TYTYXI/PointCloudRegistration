//
// Created by XI on 2022/12/7.
//
#include "Problems.hpp"
#include "problems/NullProblem.h"

#include <stdexcept>

using namespace oa;

//----------------------------ProbBase----------------------------

//----------------------------Problem----------------------------

VectorDouble Problem::fitnessScore(const VectorDouble& dv)
{
  // 检测输入维数是否与决策变量维数一致
  if (dv.size() != this->dimension()) {
    throw std::invalid_argument(u8"决策变量维数与输入vector不一致");
  }

  // 计算适度值
  VectorDouble res(this->probPtr_->fitnessScore(dv));

  // 检测输出适度值维数是否与Problem的适度值维数一致
  if (dv.size() != this->dimension()) {
    throw std::invalid_argument(u8"出适度值维数与Problem的适度值维数不一致");
  }

  return res;
}

VectorDouble::size_type Problem::dimension()
{
  return this->lowerBoundary_.size();
}

std::pair<VectorDouble, VectorDouble> Problem::bounds()
{
  return this->probPtr_->bounds();
}

void Problem::initProblemBase()
{
  auto bound = this->probPtr_->bounds();
  lowerBoundary_ = std::move(bound.first);
  upperBoundary_ = std::move(bound.second);
  numOfObjectiveFunction_ = this->probPtr_->numOfObjectiveFunction();
}

Problem::Problem()
    : Problem(NullProblem())
{
}

const VectorDouble& Problem::lowerBoundary()
{
  return lowerBoundary_;
}

const VectorDouble& Problem::upperBoundary()
{
  return upperBoundary_;
}

Problem::Problem(const Problem& other)
    : probPtr_(other.probPtr_->clone())
    , lowerBoundary_(other.lowerBoundary_)
    , upperBoundary_(other.upperBoundary_)
    , numOfObjectiveFunction_(other.numOfObjectiveFunction_)
{
}

Problem::Problem(Problem&& other) noexcept
    : probPtr_(std::move(other.probPtr_))
    , lowerBoundary_(std::move(other.lowerBoundary_))
    , upperBoundary_(std::move(other.upperBoundary_))
    , numOfObjectiveFunction_(other.numOfObjectiveFunction_)
{
}

Problem& Problem::operator=(Problem&& other) noexcept
{
  if (this != &other) {
    probPtr_ = (std::move(other.probPtr_));
    lowerBoundary_ = (std::move(other.lowerBoundary_));
    upperBoundary_ = (std::move(other.upperBoundary_));
    numOfObjectiveFunction_ = (other.numOfObjectiveFunction_);
  }
  return *this;
}

Problem& Problem::operator=(const Problem& other)
{
  return *this = Problem(other);
}

VectorDouble::size_type Problem::numOfObjectiveFunction()
{
  return probPtr_->numOfObjectiveFunction();
}
