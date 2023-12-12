#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

#include "registration_global.h"

class REGISTRATION_EXPORT MeasurementPackage
{
public:
  enum SensorType
  {
    LASER,
    RADAR,
    ELECTRONIC
  } sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;
};

#endif // MEASUREMENT_PACKAGE_H_
