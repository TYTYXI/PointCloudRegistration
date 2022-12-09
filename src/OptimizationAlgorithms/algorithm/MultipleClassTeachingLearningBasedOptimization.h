//
// Created by XI on 2022/12/6.
//

#ifndef POINTCLOUDREGISTRATION_SRC_ALGORITHM_MULTIPLECLASSTEACHINGLEARNINGBASEDOPTIMIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_ALGORITHM_MULTIPLECLASSTEACHINGLEARNINGBASEDOPTIMIZATION_H_

//#include <iostream>
//
//#include <tbb/tbb.h>
//
//#include "IOABase.h"
//
//#include "TeachingLearningBasedOptimization.h"
//
//namespace oa {
//
//template <typename TargetFunc>
//class ALGORITHM_EXPORT multipleClassTeachingLearningBasedOptimization
//    : public IOABase<typename TargetFunc::ParamsType, TargetFunc::numOfParams, TargetFunc>
//{
//
//  typedef typename TargetFunc::ParamsType ParamsType;
//  typedef IOABase<ParamsType, TargetFunc::numOfParams, TargetFunc> BaseClass;
//  typedef teachingLearningBasedOptimization<TargetFunc> XClass;
//
//public:
//  multipleClassTeachingLearningBasedOptimization(int iteration, int numOfClasses, int numOfStudents,
//                                                 TargetFunc targetFunc = TargetFunc())
//      : BaseClass(iteration, TargetFunc::numOfParams, targetFunc)
//      , numOfStudents_(numOfStudents)
//      , numOfClasses_(numOfClasses)
//  {
//    for (size_t i = 0; i < numOfClasses; ++i) {
//      xClasses_.emplace_back(new XClass(iteration / 5, numOfStudents));
//    }
//  }
//
//  void parameters(ParamsType* params) const
//  {
//    for (size_t i = 0; i < TargetFunc::numOfParams; ++i) {
//      params[i] = this->student_->params_[i];
//    }
//  }
//
//  void optimize() override
//  {
//    for (size_t i = 0; i < 4; ++i) {
//      for (size_t j = 0; j < numOfClasses_; ++j) {
//        xClasses_[j]->optimize();
//      }
//      auto head = xClasses_.begin();
//      for (size_t j = 0; j < numOfClasses_; ++j) {
//        if (head + 1 != xClasses_.end()) {
//          (*(head + 1))->replaceStudent((*(head + 1))->poorStudent(), (*head)->teacher());
//          head++;
//        } else {
//          (*xClasses_.begin())
//              ->replaceStudent((*xClasses_.begin())->poorStudent(), (*head)->teacher());
//        }
//      }
//    }
//    std::vector<typename XClass::XStudent*> teachers;
//    for (size_t i = 0; i < numOfClasses_; ++i) {
//      teachers.emplace_back(xClasses_[i]->teacher());
//    }
//    auto res = std::min_element(teachers.cbegin(), teachers.cend(),
//                                [&](XClass::XStudent* stu1, XClass::XStudent* stu2) {
//                                  return stu1->fitnessScore_ < stu2->fitnessScore_;
//                                });
//    student_ = (*res);
//  }
//
//  void setLimits(ParamsType* limits) override
//  {
//    BaseClass::setLimits(limits);
//    for (const auto& xClass : xClasses_) {
//      xClass->setLimits(limits);
//      xClass->initTLBO();
//    }
//  }
//
//private:
//  std::vector<XClass*> xClasses_;
//  typename XClass::XStudent* student_;
//
//  int numOfStudents_;
//  int numOfClasses_;
//};
//};     // namespace ioa
#endif // POINTCLOUDREGISTRATION_SRC_ALGORITHM_MULTIPLECLASSTEACHINGLEARNINGBASEDOPTIMIZATION_H_
