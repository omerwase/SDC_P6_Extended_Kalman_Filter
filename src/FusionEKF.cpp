#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225	, 		0,
        					0	, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ <<	0.09	, 		0	, 	0,
        				0	, 0.0009	, 	0,
        				0	, 		0	, 0.09;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
	
  if (!is_initialized_) {
		
		// initialize state
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
		
		// initialize state covariance matrix
		ekf_.P_ = MatrixXd(4, 4);
		ekf_.P_ << 	1	, 0	, 	0	, 	0,
								0	, 1	,		0	, 	0,
								0	, 0	,	1000	, 	0,
								0	, 0	,	 	0	, 1000;
		
		// initialize transition matrix
		ekf_.F_ = MatrixXd(4, 4);
		ekf_.F_ << 	1	, 0	, 1	, 0,
								0	, 1	, 0	, 1,
								0	, 0	, 1	, 0,
								0	, 0	, 0	, 1;

		// initialize the state ekf_.x_ with the first measurement.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			// convert radar measurement from polar to cartesian coordinates
			double rho = measurement_pack.raw_measurements_[0];
			double phi = measurement_pack.raw_measurements_[1];
			double rho_dot = measurement_pack.raw_measurements_[2];
			double px = rho * cos(phi);
			double py = rho * sin(phi);
			double vx = 0; //rho_dot * cos(phi);
			double vy = 0; //rho_dot * sin(phi);
			ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[0], 0, 0;
    }

		// initialize timestamp
		previous_timestamp_ = measurement_pack.timestamp_;
		
    // done initializing, no need to predict or update
    is_initialized_ = true;
		
		cout << "EKF initialized" << endl;
    return;
  }

	
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

	// compute the time elapsed between the current and previous measurements in seconds
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
	
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	
	// use noise_ax = 9 and noise_ay = 9
	float noise_ax = 9;
	float noise_ay = 9;
	
	// update state transition matrix F according to elapsed time (dt)
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
	
	// update the process noise covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<	dt_4/4*noise_ax	, 							0	,	dt_3/2*noise_ax	,								0,
														0	, dt_4/4*noise_ay	, 							0	, dt_3/2*noise_ay,
							dt_3/2*noise_ax	, 							0	, 	dt_2*noise_ax	, 							0,
														0	, dt_3/2*noise_ay	, 							0	, 	dt_2*noise_ay;
	
  ekf_.Predict();

	
  /*****************************************************************************
   *  Update
   ****************************************************************************/
	
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		
		// set radar measurement matrix
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		
		// set radar measurement matrix
		ekf_.R_ = R_radar_;
		
		// update the state and covariance matrices using radar measurement
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
		
  } else {
		
		// set lidar measurement matrix
		ekf_.H_ = MatrixXd(2, 4);
		ekf_.H_ << 	1, 0, 0, 0,
								0, 1, 0, 0;
		
		// set radar measurement matrix
		ekf_.R_ = R_laser_;
		
		// update the state and covariance matrices using lidar measurement
		ekf_.Update(measurement_pack.raw_measurements_);
		
  }

  // print the output
	cout << "EKF: " << endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl << endl;
}
