#ifndef LOGIC_MINIATURE_KALMAN_FILTER_H_
#define LOGIC_MINIATURE_KALMAN_FILTER_H_

#include <Eigen/Dense>


namespace opendlv {
namespace logic {
namespace miniature {

class KalmanFilter {
public:

  // State vector
  Eigen::VectorXf x_;

  // State covariance matrix
  Eigen::MatrixXf P_;

  // State Transition Matrix
  Eigen::MatrixXf F_;

  // Process Covariance Matrix;
  Eigen::MatrixXf Q_;

  // Measurement Matrix
  Eigen::MatrixXf R_;

  // bb: Hessian?
  Eigen::MatrixXf H_;

  // Control state Matrix
  Eigen::MatrixXf B_;

  // Control vector
  Eigen::VectorXf u_;

  /**
  * Constructor
  */
  KalmanFilter();

  /**
  * Destructor
  */
  virtual ~KalmanFilter();

  /**
    * Init Initializes Kalman filter
    * @param x_in Initial state
    * @param P_in Initial state covariance
    * @param F_in Transition matrix
    * @param H_in Measurement matrix
    * @param R_in Measurement covariance matrix
    * @param Q_in Process covariance matrix
  */

  void Init(Eigen::VectorXf &x_in, Eigen::MatrixXf &P_in, Eigen::MatrixXf &F_in,
      Eigen::MatrixXf &H_in, Eigen::MatrixXf &R_in, Eigen::MatrixXf &Q_in, Eigen::MatrixXf &B_in, Eigen::MatrixXf &u_in);

  void Predict();

  /**
  * Prediction Predicts the state and the state covariance
  * using the process model
  * @param delta_T Time between k and k+1 in s
  */

  void Update(const Eigen::VectorXf &z);
  /**
    * Updates the state by using standard Kalman Filter equations
    * @param z The measurement at k+1
    */

  void UpdateEKF(const Eigen::VectorXf &z);
  /**
    * Updates the state by using Extended Kalman Filter equations
    * @param z The measurement at k+1
    */

  void KF(const Eigen::VectorXf &y);
};

}
}
}

#endif /* KALMAN_FILTER_H_ */
