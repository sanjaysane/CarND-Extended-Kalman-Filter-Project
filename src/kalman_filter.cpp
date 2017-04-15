#include "kalman_filter.h"
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {

  x_ = F_ * x_; // + u - assuming u is zero (this is for control input)
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // h(x) from equation 53
  // https://d17h27t6h515a5.cloudfront.net/topher/2017/February/58b461d5_sensor-fusion-ekf-reference/sensor-fusion-ekf-reference.pdf
  double range = sqrt( pow(x_[0],2) + pow(x_[1],2) );
  double bearing;
  double range_rate;
  if (fabs(range > 0.001)) {
    bearing = atan(x_[1] / x_[0]);
    range_rate = ((x_[0] * x_[2] + x_[1] * x_[3]) / range);
  } else {
    bearing = 0;
    range_rate = 0;
  }
  MatrixXd zpred(3, 1);
  zpred << range, bearing, range_rate;

  //VectorXd y = z - H_ * x_;
  VectorXd y = z - zpred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


}
