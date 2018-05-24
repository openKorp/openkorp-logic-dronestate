#include "Eigen/Dense"
#include <string>
#include "kalman_filter.h"

namespace opendlv {
namespace logic {
namespace miniature {

class EKF_Fusion {
/*
private:
	Eigen::MatrixXf R_accelorometer_; // Measurement Covariance Matrix for the accelorometer sensor.
	Eigen::MatrixXf R_gyroscope_; // Measurement Covariance Matrix for the gyroscope sensor.
	Eigen::MatrixXf H_accelorometer_; // Measurement function for accelorometer.
	Eigen::MatrixXf H_gyroscope_; // Measurement function for gyroscope.

	float previous_timestamp_;
*/
public:

// Constructor
	EKF_Fusion();
//

// Destructor
	virtual ~EKF_Fusion();
//

	KalmanFilter ekf_; // Constructor from the Kalman Filter class.

};

}
}
}
