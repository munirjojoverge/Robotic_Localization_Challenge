/*
 * ukf.cpp
 *
 *  Uncented Kalman Filter (UKF) class.
 *  Created on: May 02, 2020
 *  Author: Munir Jojo-Verge
 */

#include "ukf.h"
#include "helper_functions.h"

#include <iostream>
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // initial state vector
  x_ = VectorXd(StateSize_);

  // initial covariance matrix
  P_ = MatrixXd(StateSize_, StateSize_);

  /*
  some studies that show that the threshold value of comfort is 1.8 m/s2, with
  medium comfort and discomfort levels of 3.6 m/s2 and 5 m/s2, respectively "W.
  J. Cheng, Study on the evaluation method of highway alignment comfortableness
  [M.S. thesis], Hebei University of Technology, Tianjin, China, 2007."
  */
  // Process noise standard deviation longitudinal acceleration in m/s^2
  double LinAcc_max = 5.0;  // max Linear acceleration expected. Borderline of
                            // disconfort. We will tune this parameter
  std_a_ = LinAcc_max / 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double AngAcc_max =
      PI / 6;  //  all IFR turns are supposed to be at "standard rate" which
               //  equals a 360 degree turn in 2 minutes (3 degrees per second)
  std_yawdd_ = AngAcc_max / 2;

  // State dimension
  n_x_ = StateSize_;

  // Augmented state dimension: This 2 comes from using the components that
  // describe the Process Noise: std_a and std_yawdd. Remeber that the reason me
  // augment the state matrix is to be able to capture the Process uncertanty
  // quantity Q and not only it's effect over the state.
  n_aug_ = n_x_ + 2;

  // Number of sigma points: We will create 2 * n_aug_ + 1 sigma points.
  // This is arbitrary and is based on intuition. We chose, for every state
  // parameter (for ex: px) 1 value that is the mean and 2 values on each side
  // if it at a distance based on Lambda. Doing this what we are trying is to
  // approximate the Non-gaussian REAL state into a gaussian state (without
  // linearizing)
  n_sig_ = 2 * n_aug_ + 1;

  // Set the predicted sigma points matrix dimentions
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // create sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;  // This value is also arbitrary and is based on
                         // experience and intuition. Once we use NIS lambda
                         // could be changed to make the filter CONSISTENT

  // Weights of sigma points
  weights_ = VectorXd(n_sig_);
  /* During the PREDICTION Step we will go through a motionUpdate of the Mean an
  Cov of the Transformed (predicted into the REAL world using f(xk,nuK) Sigma
  Points. When we aproximate this Mean and Cov, we need some weights
  (conceptually the inverse of the lambda) Since lambda is a cte (that we will
  tune) the weights are also cte and can be generated at the time of
  construction
  */
  // Initialize weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < weights_.size(); i++) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  // Measurement noise covariance matrix initialization

  R_ = MatrixXd(2, 2);
}

UKF::~UKF() {}

/**
 *
 */
void UKF::init(RobotState robotState, RobotParams robotParams) {
  // The STATE of the system represents the parameters of the object detected by
  // the sensors
  double px;   // Position coordinate X
  double py;   // Position coordinate Y
  double v;    // Velocity Magnitude
  double phi;  // Yaw angle of the vehicle wrt X axix: Angle formed by the
               // longitudinal axix across the vehicle and the inertial frame X
               // axis
  double phi_dot;  // Yaw rate or Angular velocyty of the turn. Unkown at
                   // the initialization point

  /**
  Initialize state (x_).
  */
  px = robotState.x;
  py = robotState.y;
  phi = robotState.theta;
  v = 0.0;        // Unkown at the initialization point
  phi_dot = 0.0;  // Unkown at the initialization point

  // Deal with the special case where we start at 0, 0 to avoid future div by 0.
  if (fabs(px) < std::numeric_limits<double>::epsilon() &&
      fabs(py) < std::numeric_limits<double>::epsilon()) {
    px = std::numeric_limits<double>::epsilon();
    py = std::numeric_limits<double>::epsilon();
  }
  // Assign the initial state
  x_ << px, py, v, phi, phi_dot;

  double tt = robotParams.odom_noise_translation_from_translation;
  double tr = robotParams.odom_noise_translation_from_rotation;
  double rt = robotParams.odom_noise_rotation_from_translation;
  double rr = robotParams.odom_noise_rotation_from_rotation;
  R_ << tt * tt, tr * tr, rt * rt, rr * rr;

  // done initializing, no need to predict or update
  is_initialized_ = true;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::motionUpdate(double delta_t, RobotState delta) {
  /******************
   *  Generate Sigma Points
   *******************/
  AugmentedSigmaPoints();
  // std::cout << "Sigma Points GENERATED succefully on the Augmented State" <<
  // std::endl;

  /******************
   *  Predict Sigma Points
   *******************/
  SigmaPointPrediction(delta_t);
  // std::cout << "Sigma Points PREDICTED succefully on the Augmented State" <<
  // std::endl;

  /******************
   *  Predict Mean & Cov of predicted Sigma Points
   *******************/
  PredictMeanAndCovariance();
  // std::cout << "MEAN & COVARIANCE motionUpdate succesfull" << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a measurement.
 * @param obser meas_package
 */
void UKF::sensorUpdate(std::vector<MarkerObservation> observations) {
  // set measurement dimension r, phi
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    // extract values for better readibility
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(px * px + py * py);                        // r
    Zsig(1, i) = atan2(py, px);                                  // phi
    Zsig(2, i) = (px * v1 + py * v2) / sqrt(px * px + py * py);  // r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    AngNorm(&(z_diff(1)));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  S = S + R_;

  // print result
  // std::cout << "RADAR Update " << std::endl;
  // std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  // std::cout << "S: " << std::endl << S << std::endl;
  // std::cout << "-------------" << std::endl;

  /******************
   *  Update State
   *******************/
  UpdateState(Zsig, z_pred, S, n_z, observations);
}

void UKF::AugmentedSigmaPoints() {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  // std::cout << "created augmented mean vector x_aug" << std::endl;

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  // std::cout << "created augmented state covariance P_aug" << std::endl;

  // create augmented mean state
  x_aug.head(n_x_) = x_;
  for (int i = n_x_; i < n_aug_; i++) {
    x_aug(i) = 0;
  }
  // std::cout << "created augmented mean state x_aug (Col 0) rest set to 0" <<
  // std::endl;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;
  // std::cout << "created augmented covariance matrix" << std::endl;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  // std::cout << "created square root matrix" << std::endl;

  // create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  // std::cout << "created augmented sigma points" << std::endl;
}

void UKF::SigmaPointPrediction(double dt) {
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    // Precalculate sin, cos and dt2 for optimization
    double sin_yaw = sin(yaw);
    double cos_yaw = cos(yaw);
    double dt2 = dt * dt;

    double yaw_p = yaw + yawd * dt;
    double v_p = v;
    double yawd_p = yawd;

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > std::numeric_limits<double>::epsilon()) {
      px_p = p_x + v / yawd * (sin(yaw_p) - sin_yaw);
      py_p = p_y + v / yawd * (cos_yaw - cos(yaw_p));
    } else {
      px_p = p_x + v * dt * cos_yaw;
      py_p = p_y + v * dt * sin_yaw;
    }

    // add noise
    px_p = px_p + 0.5 * nu_a * dt2 * cos_yaw;
    py_p = py_p + 0.5 * nu_a * dt2 * sin_yaw;
    v_p = v_p + nu_a * dt;

    yaw_p = yaw_p + 0.5 * nu_yawdd * dt2;
    yawd_p = yawd_p + nu_yawdd * dt;

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance() {
  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    AngNorm(&(x_diff(3)));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  // print result
  // std::cout << "Predicted state" << std::endl;
  // std::cout << x_ << std::endl;
  // std::cout << "Predicted covariance matrix" << std::endl;
  // std::cout << P_ << std::endl;
}

/**
*  Angle normalization to [-Pi, Pi]
        For optimization we create this procedure since it's repeated twice
*/
void UKF::AngNorm(double* ang) {
  while (*ang > PI) *ang -= 2. * PI;
  while (*ang < -PI) *ang += 2. * PI;
}

void UKF::UpdateState(MatrixXd Zsig, MatrixXd z_pred, MatrixXd S, int n_z,
                      std::vector<MarkerObservation> observations) {
  // Measurements
  MarkerObservation* ptr = &observations[0];
  Map<VectorXd> z(ptr, observations.size());

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    AngNorm(&(z_diff(1)));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    AngNorm(&(x_diff(3)));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  AngNorm(&(z_diff(1)));

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS

  NIS_ = z.transpose() * S.inverse() * z;

  // print result
  // print result
  // std::cout << "FINAL UPDATE " << std::endl;
  // std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  // std::cout << "Updated state covariance P: " << std::endl << P_ <<
  // std::endl;
}