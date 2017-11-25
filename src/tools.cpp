#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if (estimations.size() == 0) {
		cout << "Error: estimations vector is empty" << endl;
	}
	if (estimations.size() != ground_truth.size()) {
		cout << "Error: estimations and ground_truth vectors are not the same size" << endl;
	}
	
	//accumulate squared residuals
	VectorXd squared_sum;
	squared_sum.setZero(estimations[0].size());
	for(int i=0; i < estimations.size(); ++i){
		VectorXd res = estimations[i] - ground_truth[i];
		squared_sum = squared_sum.array() + res.array().pow(2);
	}
	
	//calculate the mean
	VectorXd mean = squared_sum.array() / estimations.size();
	
	//calculate the squared root
	rmse = mean.array().sqrt();
	
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	
	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);
	
	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "ERROR: division by zero in CalculateJacobian()" << endl;
		return Hj;
	}
	
	//compute the Jacobian matrix
	Hj << 							(px/c2)	, 								(py/c2)	, 		0	, 			0,
										-(py/c1)	, 								(px/c1)	, 		0	, 			0,
				py*(vx*py - vy*px)/c3	, 	px*(px*vy - py*vx)/c3	, px/c2	, 	py/c2;
	
	return Hj;
}
