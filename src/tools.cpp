#include "tools.h"
#include <iostream>

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
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
      std::cout << "invalid inputs.";
      return rmse;
  }

  
  VectorXd r(4);
  r << 0,0,0,0;
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // TODO: calculate the mean
  rmse = rmse / estimations.size();
  
  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  
  float px_2 = pow(px,2.0);
  float py_2 = pow(py,2.0);
  float pxy_2 = px_2 + py_2;
  float pxy_2_sqrt = pow(pxy_2, 0.5);
  float pxy_2_3_2 = pow(pxy_2,3.0/2.0);

  // check division by zero
  if (px_2 + py_2 == 0.0) {
    std::cout << "Error: can not divide by zero.";
    return Hj;
  }
  
  // compute the Jacobian matrix
  Hj << px/pxy_2_sqrt, py/pxy_2_sqrt, 0, 0,
        -py/pxy_2, px/pxy_2, 0, 0,
        py*(vx*py-vy*px)/pxy_2_3_2, px*(vy*px-vx*py)/pxy_2_3_2,px/pxy_2_sqrt, py/pxy_2_sqrt;

  return Hj;
}
