////
//// Created by XI on 2022/12/1.
////
//
//#ifndef POINTCLOUDREGISTRATION_SRC_ALGORITHM_ARTIFICIALBEECOLONY_H_
//#define POINTCLOUDREGISTRATION_SRC_ALGORITHM_ARTIFICIALBEECOLONY_H_
//
//#include "IOABase.h"
//#include <iostream>
//
//namespace ioa {
//
//template <typename TargetFunc>
//class ALGORITHM_EXPORT ArtificialBeeColony
//    : public IOABase<typename TargetFunc::ParamsType, TargetFunc::numOfParams, TargetFunc>
//{
//  typedef TargetFunc::ParamsType ParamsType;
//  typedef IOABase<ParamsType, TargetFunc::numOfParams, TargetFunc> BaseClass;
//
//  template <typename ParamsType, int numOfParams>
//  class Bee
//  {
//  public:
//    Bee(ArtificialBeeColony* tlbo, double fitnessScore = 0.0)
//        : tlbo_(tlbo)
//    {
//    }
//
//  public:
//    ArtificialBeeColony* tlbo_;
//    ParamsType params_[TargetFunc::numOfParams];
//    double fitnessScore_;
//  };
//
//  typedef Bee<ParamsType, TargetFunc::numOfParams> XBee;
//
//public:
//  ArtificialBeeColony(int iteration, int numOfStudents, TargetFunc targetFunc = TargetFunc())
//      : BaseClass(iteration, TargetFunc::numOfParams, targetFunc)
//  {
//  }
//
//  double targetFunctionValue()
//  {
//    return this->fitnessScore_;
//  }
//
//  void optimize() override
//  { // 初始化学生
//    for (size_t i = 0; i <numOfBees; ++i) {
//      students_.emplace_back(new(this));
//      this->init(this->students_.back()->params_);
//
//      //      std::cout << this->students_.back()->params_[0] << std::endl;
//      //      std::cout << this->students_.back()->params_[1] << std::endl;
//    }
//    for (size_t i = 0; i < numOfStudents_; ++i) {
//      students_[i]->fitnessScore_ = this->targetFunc_(students_[i]->params_);
//    }
//    for (size_t i = 0; i < this->iteration_; ++i) {
//      calTheMean();
//      calTheTeacher();
//      teaching();
//      learning();
//    }
//    calTheTeacher();
//    this->fitnessScore_ = this->targetFunc_(teacher_);
//  }
//};
//
//} // namespace ioa
//#endif // POINTCLOUDREGISTRATION_SRC_ALGORITHM_ARTIFICIALBEECOLONY_H_
