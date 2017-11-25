#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  // kalman filter prediction
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	
	/* used for debugging
	std::cout << "Pre Update: " << std::endl;
	std::cout << "x_ = " << x_ << std::endl;
	*/
	
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Sinv = S.inverse();
  MatrixXd K = P_ * Ht * Sinv;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	// current estimate
	double px = x_(0);
	double py = x_(1);
	double vx = x_(2);
	double vy = x_(3);
	
	/* used for debugging
	std::cout << "Pre UpdateEKF: " << std::endl;
	std::cout << "x_ = " << x_ << std::endl;
	std::cout << "H_ = " << H_ << std::endl;
	*/
	
	// convert to polar coordinates
	double rho = sqrt(px*px + py*py);
	double phi = atan2(py, px);
	double rho_dot = (px*vx + py*vy) / rho;

	// extended kalman filter update
	VectorXd hx(3);
	hx << rho, phi, rho_dot;
	VectorXd y = z - hx;
	
	// normalize phi between [-PI, PI];
	const double PI = 3.141592653589793;
	if (y(1) > PI) y(1) -= 2*PI;
	else if (y(1) < -PI) y(1) += 2*PI;
	
	// H_ should be the jacobian Hj, initialized through FusionEKF.cpp
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Sinv = S.inverse();
	MatrixXd K = P_ * Ht * Sinv;
	
	// new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
	
	/* used for debugging
	std::cout << "UpdateEKF: " << std::endl;
	std::cout << "rho: " << rho << std::endl;
	std::cout << "phi: " << phi << std::endl;
	std::cout << "rho_dot: " << rho_dot << std::endl;
	*/
}
