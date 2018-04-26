// Author: Marios Aspris

#include <iostream>
#include "Eigen/Dense"
#include "EKF_Fusion.h"

namespace opendlv {
namespace logic {
namespace miniature {

// Constructor.
EKF_Fusion::EKF_Fusion()
	:	ekf_() {

	// Covariance and Measurement matrices initialization.
	// R_accelorometer_ = Eigen::MatrixXf(); // Matrix sizes(?)
	// R_gyroscope_ = Eigen::MatrixXf();
	// H_accelerometer_ = Eigen::MatrixXf();
	// H_gyroscope_ = Eigen::MatrixXf();


	// Comments for the implementation below.
	// ekf_.x_ = Eigen::VectorXd(12,1);
	// ekf_.u_ = Eigen::VectorXd(4,1);
	// ekf_.x_ = (x, y, z, yaw, pitch, roll, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot);
	// ekf_.u_ = (u_1, u_2, u_3, u_4);

	float m = 1.5; 			//(?) mass of the quadcopter in kg.
	float I_x = 1.2416; // This is suggested in the paper.
	float I_y = 1.2416; // This is suggested in the paper.
	float I_z = 1.2416; // This is suggested in the paper.
	float d = 0.24;     // Distance of the propellers from the center. Distance measured in meters.
	float dt = 0.125;  	// Time between inputs. Static.

	/* ---- Note: Comma initialization of matrices. ---- */

	// State transition Matrix. 6 degrees of freedom.
	ekf_.F_ = Eigen::MatrixXf(12, 12);
	ekf_.F_ << 0,0,0,0,       0,       		0, 1*dt, 0,0,0, 0,0,   	  // x
	           0,0,0,0,       0,	     		0,0, 1*dt, 0,0, 0,0,			// y
         	   0,0,0,0,       0,	     		0,0,0, 1*dt, 0, 0,0,			// z
			   		 0,0,0,0,		    0,	     		0, 0,0,0,0, 0, 1*dt,  		// yaw
			   		 0,0,0,0,       0,       		0,0, 0,0,0, 1*dt, 0,      // pitch
			    	 0,0,0,0,       0,       		0,0,0, 0, 1*dt, 0,0,			// roll
			    	 0,0,0,0, -9.7145f*dt, -1.3653f*dt, 0,0,0,0, 0, 0,			// x_speed
			    	 0,0,0,0, -1.3653f*dt,  9.7145f*dt,0,0, 0, 0, 0,0,			// y_speed
			    	 0,0,0,0,       0,      			 0, 0,0, 0, 0,0,0,			// z_speed
			    	 0,0,0,0,		    0,	     			 0,0, 0, 0,0, 0,0,			// yaw_speed
			    	 0,0,0,0,	      0,       			 0,0,0, 0, 0, 0,0,			// pitch_speed
			    	 0,0,0,0,		    0,       			 0,0,0,0, 0, 0, 0;			// roll_speed


	// Control Matrix. 4 controls.
	ekf_.B_ = Eigen::MatrixXf(12,4);
	ekf_.B_ << 0				 		 ,0,0,0,
			   0				 				 ,0,0,0,
			   0				         ,0,0,0,
			   0				 				 ,0,0,0,
			   0				 				 ,0,0,0,
			   0				  			 ,0,0,0,
			   0				 				 ,0,0,0,
			   0				 				 ,0,0,0,
			   -1/m              ,0,0,0,
			   0,   d/I_x          ,0,0,
			   0,0,       d/I_y      ,0,
			   0,0,0,             1/I_z;


/* vel_noise_std = 0.05;
   pos_noise_std = 0.05;
*/
	ekf_.P_ = Eigen::MatrixXf::Identity(12, 12); // Initial Covariance Matrix

	// Sensor Measurement noise.
	ekf_.R_ = Eigen::MatrixXf::Identity(12, 12); // Sensor measurement noise matrix

	// Measurement Matrix.
	ekf_.H_ = Eigen::MatrixXf(3, 12);
	ekf_.H_ << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

	ekf_.Q_ = Eigen::MatrixXf(12, 12);
	ekf_.Q_ << 0.01,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Variance x_axis
			  		0, 0.01,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Variance y axis
			  		0, 0, 0.01,   0, 0, 0, 0, 0, 0, 0, 0, 0,			// Variance z axis
			  		0, 0, 0, 0.001,  0, 0, 0, 0, 0, 0, 0, 0,			// Variance x_dot axis
			  		0, 0, 0, 0, 0.001,  0, 0, 0, 0, 0, 0, 0,			// Variance y_dot axis
			  		0, 0, 0, 0, 0, 0.001,  0, 0, 0, 0, 0, 0,      // Variance z_dot axis
			  		0, 0, 0, 0, 0, 0, 5.0e-6, 0, 0, 0, 0, 0,			// Variance roll       ( Phi )
			  		0, 0, 0, 0, 0, 0, 0, 5.0e-6, 0, 0, 0, 0,			// Variance pitch      ( Theta )
			  		0, 0, 0, 0, 0, 0, 0, 0, 5.0e-6, 0, 0, 0,			// Variance yaw        ( Psi )
			  		0, 0, 0, 0, 0, 0, 0, 0, 0, 5.0e-7, 0, 0,			// Variance roll_dot   ( Phi_dot )
			  		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5.0e-7, 0,			// Variance pitch_dot  ( Theta_dot )
			  		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5.0e-7;			// Variance yaw_dot	   ( Psi_dot )

}

// Destructor.
EKF_Fusion::~EKF_Fusion() {}

}
}
}
