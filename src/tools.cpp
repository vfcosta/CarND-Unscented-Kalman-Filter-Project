#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

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
  if(estimations.size()==0 || estimations.size()!=ground_truth.size()) {
    cout << "Invalid input " << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

VectorXd Tools::ConvertPolarToCartesian(const Eigen::VectorXd& measurements) {
  float p = measurements[0];
  float phi = measurements[1];
  float v = measurements[2];
  float cos_phi = cos(phi);
  float sin_phi = sin(phi);
  VectorXd cartesian = VectorXd(5);
  cartesian << p * cos_phi, p * sin_phi, 0, 0, 0;
  return cartesian;
}