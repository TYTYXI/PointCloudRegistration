//
// Created by XI on 2023/10/17.
//

#ifndef POINTCLOUDREGISTRATION_SRC_REGISTRATION_FILTER_POINTCLOUDKF_H_
#define POINTCLOUDREGISTRATION_SRC_REGISTRATION_FILTER_POINTCLOUDKF_H_

#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include <fstream>
#include <string>
#include <vector>

#include "registration_global.h"

class REGISTRATION_EXPORT PointCloudKF
{
public:
  /**
   * Constructor.
   */
  PointCloudKF(double init_x, double init_y, double init_z);

  /**
   * Destructor.
   */
  virtual ~PointCloudKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage& measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
};

#endif // POINTCLOUDREGISTRATION_SRC_REGISTRATION_FILTER_POINTCLOUDKF_H_
