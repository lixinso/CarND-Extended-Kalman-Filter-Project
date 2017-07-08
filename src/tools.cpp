#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	std:cout<< estimations.size() << std::endl;


	VectorXd rmse(4);
	rmse << 0,0,0,0;

	if(estimations.size() == 0 || ground_truth.size() == 0 || ground_truth.size() != estimations.size()){
		std::cout<<"Error: Calculate RMSE error. Parameters are wrong" << std::endl;
		return rmse;
	}

	for(int i = 0; i < estimations.size(); i++){
		VectorXd res = estimations[i] - ground_truth[i];
		res = res.array() * res.array();
		rmse += res;
	}
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

	MatrixXd Hj(3,4);
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float c1 = px*px + py*py;
	float c2 = sqrt(c1);
	float c3 = sqrt(c1*c2);

	if(fabs(c1)< 0.0000001){
		std::cout<<"Error: CalculateJacobian. Divided by zero exception." << std::endl;
		return Hj;
	}

	Hj << (px / c2), (py / c2), 0, 0,
			-(py / c1), (px/c1), 0, 0,
			py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

	return Hj;

}
