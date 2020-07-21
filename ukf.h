/*
 * ukf.h
 *
 *  Uncented Kalman Filter (UKF) class.
 *  Created on: May 02, 2020
 *  Author: Munir Jojo-Verge
 */

#ifndef UKF_H
#define UKF_H

#include <fstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "robot_defs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
 public:
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* state vector: [x y vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in microseconds
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Number of Sigma Points to generate
  int n_sig_;

  ///* Sigma point spreading parameter
  double lambda_;

  MatrixXd R_;
  MatrixXd H_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  ~UKF();

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const { return is_initialized_; }

  /**
   * Initialize the KF matrices and parameters
   * @param
   * Initialize the KF matrices and parameters
   */
  void init(RobotState robotState, RobotParams robotParams,
            FieldLocation markerLocations[NUM_LANDMARKS]);

  /**
   * motionUpdate Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void motionUpdate(double delta_t, RobotState delta);

  /**
   * Updates the state and the state covariance matrix using a the measurement
   * @param meas_package The measurement at k+1
   */
  void sensorUpdate(std::vector<MarkerObservation> observations);

 private:
  // This is the number of States that our System have. This way I can avoid
  // hard coded values and make it more modular and easy to change
  const int StateSize_ = 5;
  /*
  The states are: (The STATE of the system represents the parameters of the
  object detected by the sensors) px: Position coordinate X py: Position
  coordinate Y v:  Velocity Magnitude phi: Yaw angle of the vehicle wrt X axix:
  Angle formed by the longitudinal axix across the vehicle and the inertial
  frame X axis phi_dot: Yaw rate or Angular velocyty of the turn. Unkown at the
  initialization point
  */

  // create sigma point matrix
  MatrixXd Xsig_aug_;

  ///* NIS
  double NIS_;

  void AugmentedSigmaPoints();
  void SigmaPointPrediction(double dt);
  void PredictMeanAndCovariance();

  /**
   *  Angle normalization to [-Pi, Pi]
   *  @param angle
   */
  void AngNorm(double* ang);

  void UpdateState(MatrixXd Zsig, MatrixXd z_pred, MatrixXd S, int n_z,
                   std::vector<MarkerObservation> observations);
};

#endif /* UKF_H */
