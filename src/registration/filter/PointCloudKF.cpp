//
// Created by XI on 2023/10/17.
//
#include "Eigen/Dense"

#include "PointCloudKF.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
PointCloudKF::PointCloudKF(double init_x, double init_y, double init_z)
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(3, 3);
  R_laser_.setIdentity();
  R_laser_(0, 0) = 1.12;
  R_laser_(0, 0) = 1.12;
  R_laser_(0, 0) = 1.12;
  H_laser_ = MatrixXd(3, 6);
  H_laser_.setIdentity();
  H_laser_(0, 0) = 1;
  H_laser_(1, 1) = 1;
  H_laser_(2, 2) = 1;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  // Initialize P
  ekf_.P_ = MatrixXd(6, 6);
  ekf_.P_.setZero();
  ekf_.P_(0, 0) = init_x;
  ekf_.P_(1, 1) = init_y;
  ekf_.P_(2, 2) = init_z;
}

/**
 * Destructor.
 */
PointCloudKF::~PointCloudKF()
{
}

void PointCloudKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
//    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(6);
    ekf_.x_ << 1, 1, 1, 1, 1, 1;

    ekf_.F_ = MatrixXd(6, 6);
    ekf_.F_ << 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::ELECTRONIC) {
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      ekf_.x_(2) = measurement_pack.raw_measurements_(2);
    }

    // done initializing, no need to predict or update
//    cout << "EKF init: " << ekf_.x_ << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 3) = dt;
  ekf_.F_(1, 4) = dt;
  ekf_.F_(2, 5) = dt;

  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float noise_az = 9.0;

  double dt2 = dt * dt;
  double dt3 = dt * dt * dt;
  double dt4 = dt * dt * dt * dt;

  MatrixXd G = MatrixXd(6, 3);
  G.setZero();
  G(0, 0) = 0.5 * dt2;
  G(1, 1) = 0.5 * dt2;
  G(2, 2) = 0.5 * dt2;
  G(3, 0) = dt;
  G(4, 1) = dt;
  G(5, 2) = dt;
  MatrixXd noiseMatrix = MatrixXd(3, 3);
  noiseMatrix.setZero();
  noiseMatrix(0, 0) = noise_ax * noise_ax;
  noiseMatrix(1, 1) = noise_ay * noise_ay;
  noiseMatrix(2, 2) = noise_az * noise_az;
  ekf_.Q_ = MatrixXd(6, 6);
  ekf_.Q_ = G * noiseMatrix * G.transpose();

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::ELECTRONIC) {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
//  cout << "x_ = " << ekf_.x_ << endl;
//  cout << "P_ = " << ekf_.P_ << endl;
}
