/*
 * linear_kalman_filter.h
 *
 *  Created on: Oct 2, 2016
 *      Author: aicrobo
 */

#ifndef INCLUDE_LINEAR_KALMAN_FILTER_H_
#define INCLUDE_LINEAR_KALMAN_FILTER_H_

#include <Eigen/Core>
#include <Eigen/LU>
#include <boost/shared_ptr.hpp>

//! Variable names reflect symbols used in the Kalman filter in Probabilistic Robotics.
class LinearKalmanFilter {
 public:
  int id_;
  //! The time associated with the most recent prediction or update of the filter.
  double timestamp_;
  //! The state mean.
  Eigen::VectorXd mu_;
  //! The state covariance.
  Eigen::MatrixXd sigma_;
  Eigen::MatrixXd measurement_matrix_;
  Eigen::MatrixXd transition_covariance_;
  Eigen::MatrixXd measurement_covariance_;
  Eigen::MatrixXd kalman_gain_;

  LinearKalmanFilter(int id, double timestamp,
             const Eigen::VectorXd& initial_state,
             const Eigen::MatrixXd& initial_sigma,
             const Eigen::MatrixXd& measurement_matrix,
             const Eigen::MatrixXd& transition_covariance,
             const Eigen::MatrixXd& measurement_covariance);

  //! Returns the product of the x and y position variances.
  double getPositionUncertainty();

  //! Estimates the next state of the tracked object based on the transition matrix.
  //! Lines 2 and 3 of Kalman_filter algorithm in Probabilistic Robotics, Chapter 3.
  void predict(const Eigen::MatrixXd& transition_matrix, double timestamp);

  //! Estimates the state of the tracked object based on a given measurement.
  //! Lines 4-7 of Kalman_filter algorithm in Probabilistic Robotics, Chapter 3.
  void update(const Eigen::VectorXd& measurement, double timestamp);
};

#endif //LINEAR_KALMAN_FILTER_H



#endif /* INCLUDE_LINEAR_KALMAN_FILTER_H_ */
