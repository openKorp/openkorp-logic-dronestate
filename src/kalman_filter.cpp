#include <Eigen/Dense>

#include <kalman_filter.h>

using namespace Eigen;
//
namespace opendlv {
namespace logic {
namespace miniature {


KalmanFilter::KalmanFilter()
    : x_()
    , P_()
    , F_()
    , Q_()
    , R_()
    , H_()
    , B_()
    , u_()
{}

KalmanFilter::~KalmanFilter()
{}

void KalmanFilter::Init(Eigen::VectorXf &x_in, Eigen::MatrixXf &P_in, Eigen::MatrixXf &F_in,
      Eigen::MatrixXf &H_in, Eigen::MatrixXf &R_in, Eigen::MatrixXf &Q_in, Eigen::MatrixXf &B_in, Eigen::MatrixXf &u_in)
{

  x_ = x_in;  // State Vector
  P_ = P_in;  // State Covariance Matrix
  F_ = F_in;  // State transition Matrix
  H_ = H_in;  // Measurement Matrix
  R_ = R_in;  // Measurement Covariance Matrix/ uncertainty Matrix //Reads the sensor
  Q_ = Q_in;  // Process Covariance Matrix / uncertainty Matrix
  B_ = B_in;  // State matrix for contol.
  u_ = u_in;  // Equations for controlling from observations of the environment


}

void KalmanFilter::Predict() {
  // x_dot = F_ * x_ + B_ * u_.  Predicting next state.
  x_ = F_ * x_ + B_ * u_; // Need to add control with B * u if I have obstacles to avoid.
  // MatrixXd Ft = F_.transpose();
  // P_ = F_ * P_ * Ft + Q_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXf &z) {

  VectorXf y = z - H_ * x_;
  KF(y);
}

void KalmanFilter::KF(const VectorXf &y) {

  MatrixXf Ht = H_.transpose();
  MatrixXf S = H_ * P_ * Ht + R_;
  //MatrixXd Si = S.inverse();
  MatrixXf K = P_ * Ht * (S.inverse());

  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXf I = MatrixXf::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

}
}
}
