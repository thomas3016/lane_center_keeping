/*
 * linear_kalman_filter.cpp
 *
 *  Created on: Oct 2, 2016
 *      Author: aicrobo
 */

#include <lane_center_keeping/linear_kalman_filter.h>

using namespace Eigen;
using boost::shared_ptr;

LinearKalmanFilter::LinearKalmanFilter(int id, double timestamp,
                       const Eigen::VectorXd& initial_state,
                       const Eigen::MatrixXd& initial_sigma,
                       const Eigen::MatrixXd& measurement_matrix,
                       const Eigen::MatrixXd& transition_covariance,
                       const Eigen::MatrixXd& measurement_covariance) :
  id_(id),
  timestamp_(timestamp),
  mu_(initial_state),
  sigma_(initial_sigma),
  measurement_matrix_(measurement_matrix),
  transition_covariance_(transition_covariance),
  measurement_covariance_(measurement_covariance),
  kalman_gain_(MatrixXd::Zero(initial_state.rows(), measurement_covariance.cols()))
{
}


void LinearKalmanFilter::predict(const Eigen::MatrixXd& transition_matrix, double timestamp) {
  mu_ = transition_matrix * mu_;
  sigma_ = transition_matrix * sigma_ * transition_matrix.transpose() + transition_covariance_;
  timestamp_ = timestamp;
}

void LinearKalmanFilter::update(const Eigen::VectorXd& measurement, double timestamp) {
  MatrixXd tmp = measurement_matrix_ * sigma_ * measurement_matrix_.transpose() + measurement_covariance_;
  kalman_gain_ = sigma_ * measurement_matrix_.transpose() * tmp.inverse();
  mu_ = mu_ + kalman_gain_ * (measurement - measurement_matrix_ * mu_);
  sigma_ = (MatrixXd::Identity(mu_.rows(), mu_.rows()) - kalman_gain_ * measurement_matrix_) * sigma_;
  timestamp_ = timestamp;
}

double LinearKalmanFilter::getPositionUncertainty() {
  return sigma_(0, 0) * sigma_(1, 1);
}


