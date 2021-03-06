#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.*/
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() != 0 && estimations.size() == ground_truth.size()) {
   	for (int i = 0; i < estimations.size(); ++i) {
    	VectorXd residual = estimations[i] - ground_truth[i];
      	residual = residual.array() * residual.array();
      	rmse += residual;
    }
    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();
  }
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float h11, h12, h21, h22, h31, h32, h33, h34;

  // TODO: YOUR CODE HERE 
  if (px != 0 && py != 0) {
    h11 = px / sqrt(pow(px, 2) + pow(py, 2));
    h12 = py / sqrt(pow(px, 2) + pow(py, 2));
    h21 = -py / (pow(px, 2) + pow(py, 2));
    h22 = px / (pow(px, 2) + pow(py, 2));
    h31 = (py * (vx*py - vy*px)) / pow(pow(px, 2) + pow(py, 2), 3/2);
    h32 = (px * (vy*px - vx*py)) / pow(pow(px, 2) + pow(py, 2), 3/2);
    h33 = px / sqrt(pow(px, 2) + pow(py, 2));
    h34 = py / sqrt(pow(px, 2) + pow(py, 2)); 
  }
  
  Hj << h11, h12, 0, 0,
        h21, h22, 0, 0,
        h31, h32, h33, h34;
  return Hj;
}
