//
// Created by XI on 2022/11/29.
//

#ifndef POINTCLOUDREGISTRATION_SRC_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATION_H_
#define POINTCLOUDREGISTRATION_SRC_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATION_H_

#include <iostream>

#include "IOABase.h"

namespace ioa {

template <typename TargetFunc>
class ALGORITHM_EXPORT teachingLearningBasedOptimization
    : public IOABase<typename TargetFunc::ParamsType, TargetFunc::numOfParams, TargetFunc>
{
  typedef typename TargetFunc::ParamsType ParamsType;
  typedef IOABase<ParamsType, TargetFunc::numOfParams, TargetFunc> BaseClass;

public:
  teachingLearningBasedOptimization(int iteration, int numOfStudents,
                                    TargetFunc targetFunc = TargetFunc())
      : BaseClass(iteration, TargetFunc::numOfParams, targetFunc)
      , numOfStudents_(numOfStudents)
  {
  }

  template <typename ParamsType, int numOfParams>
  class Student
  {
  public:
    Student(teachingLearningBasedOptimization* tlbo, double fitnessScore = 0.0)
        : tlbo_(tlbo)
    {
    }

  public:
    teachingLearningBasedOptimization* tlbo_;
    ParamsType params_[TargetFunc::numOfParams];
    double fitnessScore_;
  };

  typedef Student<ParamsType, TargetFunc::numOfParams> XStudent;

  void setNumOfStudents(int numOfStudents)
  {
    numOfStudents_ = numOfStudents;
  }

  void initTLBO()
  {
    // 初始化学生
    for (size_t i = 0; i < numOfStudents_; ++i) {
      students_.emplace_back(new XStudent(this));
      this->init(this->students_.back()->params_);

      //      std::cout << this->students_.back()->params_[0] << std::endl;
      //      std::cout << this->students_.back()->params_[1] << std::endl;
    }
    for (size_t i = 0; i < numOfStudents_; ++i) {
      students_[i]->fitnessScore_ = this->targetFunc_(students_[i]->params_);
    }
  }

  double targetFunctionValue()
  {
    return this->fitnessScore_;
  }

  void parameters(ParamsType* params) const
  {
    for (size_t i = 0; i < TargetFunc::numOfParams; ++i) {
      params[i] = this->teacher_[i];
    }
  }

  void replaceStudent(XStudent* oldStu, XStudent* newStu)
  {
    auto tempStu = new XStudent(this);
    for (size_t i = 0; i < TargetFunc::numOfParams; ++i) {
      tempStu->params_[i] = newStu->params_[i];
    }
    tempStu->fitnessScore_=this->targetFunc_(tempStu->params_);

    auto res = std::find_if(this->students_.begin(), this->students_.end(),
                            [&](XStudent* stu) { return stu == oldStu; });
    delete (*res);
    (*res) = tempStu;
  }

  XStudent* teacher()
  {
    return *std::min_element(students_.begin(), students_.end(),
                             [](XStudent* student1, XStudent* student2) {
                               return student1->fitnessScore_ < student2->fitnessScore_;
                             });
  }

  XStudent* poorStudent()
  {
    return *std::max_element(students_.begin(), students_.end(),
                             [](XStudent* student1, XStudent* student2) {
                               return student1->fitnessScore_ < student2->fitnessScore_;
                             });
  }

  void optimize() override
  {
    for (size_t i = 0; i < this->iteration_; ++i) {
      calTheMean();
      calTheTeacher();
      teaching();
      learning();
    }
    calTheTeacher();
    this->fitnessScore_ = this->targetFunc_(teacher_);
  }

private:
  void calTheMean()
  {
    ParamsType mean[TargetFunc::numOfParams];
    for (size_t i = 0; i < this->numOfParams_; ++i) {
      mean[i] = 0;
    }
    for (size_t i = 0; i < students_.size(); ++i) {
      for (size_t j = 0; j < this->numOfParams_; ++j) {
        mean[j] += students_[i]->params_[j];
      }
    }
    for (size_t i = 0; i < this->numOfParams_; ++i) {
      mean_[i] = mean[i] / static_cast<ParamsType>(this->numOfStudents_);
    }
  }

  void calTheTeacher()
  {
    auto teacher = std::min_element(students_.begin(), students_.end(),
                                    [](XStudent* student1, XStudent* student2) {
                                      return student1->fitnessScore_ < student2->fitnessScore_;
                                    });
    for (size_t i = 0; i < this->numOfParams_; ++i) {
      teacher_[i] = (*teacher)->params_[i];
      //            std::cout << (*teacher)->params_[i] << std::endl;
    }
  }

  void teaching()
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> r(0.0, 1.0);
    std::uniform_int_distribution<int> t(1, 2);

#pragma omp parallel for
    for (int j = 0; j < numOfStudents_; ++j) {
      //      std::cout << omp_get_num_threads() << std::endl;
      float R;
      float T;
      bool flag = true;
      while (flag) {
        R = r(gen);
        T = (float)t(gen);
        //                std::cout << R << "  "<<T<<"  "<<std::endl;
        ParamsType tempVariables[TargetFunc::numOfParams];
        //        float tempVariables[2];
        for (size_t k = 0; k < this->numOfParams_; ++k) {
          tempVariables[k] = students_[j]->params_[k] + R * (teacher_[k] - T * mean_[k]);
        }

        if (auto res = this->subjectToConstrains(tempVariables); !res) {
          //          std::cout << res << std::endl;
          continue;
        }

        if (auto res = this->targetFunc_(tempVariables); res < students_[j]->fitnessScore_) {
          for (int k = 0; k < TargetFunc::numOfParams; ++k) {
            students_[j]->params_[k] = tempVariables[k];
          }
          students_[j]->fitnessScore_ = res;
          //                    std::cout << "teaching stage student" << j
          //                              << " success fitness_score = " <<
          //                              students_[j]->fitnessScore_ << std::endl;
        } else {
          //                    std::cout << "teaching stage student" << j
          //                              << " fail fitness_score = " << students_[j]->fitnessScore_
          //                              << std::endl;
        }
        flag = false;
      }
    }
  }

  void learning()
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> r(0.0, 1.0);
    std::uniform_int_distribution<int> stu2(0, this->numOfStudents_ - 1);
#pragma omp parallel for
    for (int j = 0; j < numOfStudents_; ++j) {
      auto student2 = stu2(gen);

      while (student2 == j) {
        student2 = stu2(gen);
      }

      bool flag = true;
      float R;

      while (flag) {
        ParamsType tempVariables[TargetFunc::numOfParams];
        R = r(gen);
        if (students_[j] < students_[student2]) {
          for (int k = 0; k < TargetFunc::numOfParams; ++k) {
            tempVariables[k] = students_[j]->params_[k] +
                               R * (students_[j]->params_[k] - students_[student2]->params_[k]);
          }
        } else {
          for (int k = 0; k < TargetFunc::numOfParams; ++k) {
            tempVariables[k] = students_[j]->params_[k] +
                               R * (students_[student2]->params_[k] - students_[j]->params_[k]);
          }
        }
        if (auto res = this->subjectToConstrains(tempVariables); !res) {
          //          std::cout << res << std::endl;
          continue;
        }

        if (auto res = this->targetFunc_(tempVariables); res < students_[j]->fitnessScore_) {
          for (int k = 0; k < TargetFunc::numOfParams; ++k) {
            students_[j]->params_[k] = tempVariables[k];
          }
          students_[j]->fitnessScore_ = res;
          //                    std::cout << "learning stage student" << j
          //                              << " success fitness_score = " <<
          //                              students_[j]->fitnessScore_ << std::endl;
        } else {
          //                    std::cout << "learning stage student" << j
          //                              << " fail fitness_score = " << students_[j]->fitnessScore_
          //                              << std::endl;
        }
        flag = false;
      }
    }
  }

private:
  std::vector<XStudent*> students_;
  int numOfStudents_;

  ParamsType mean_[TargetFunc::numOfParams];
  ParamsType teacher_[TargetFunc::numOfParams];
};

} // namespace ioa

#endif // POINTCLOUDREGISTRATION_SRC_ALGORITHM_TEACHINGLEARNINGBASEDOPTIMIZATION_H_
