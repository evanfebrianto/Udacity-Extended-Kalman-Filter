#include "tools.h"
#include <iostream>
#include <cmath>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;
using namespace std;


Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
     * TODO: Calculate the RMSE here.
     */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Empty estimation or ground_truth data does not match estimation size!" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();
  
//   cout << CalculateRMSE(estimations, ground_truth) << endl;
  for (int i = 0; i < rmse.size(); i++) 
    cout << rmse[i] << endl;

  // return the result
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

  px = convertNearZero(px);
  py = convertNearZero(py);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
  -(py/c1), (px/c1), 0, 0,
  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;

}

float Tools::convertNearZero(float &state_val) {
  state_val = (fabs(state_val) < NEAR_ZERO) ? NEAR_ZERO : state_val;
  return state_val;
}

Eigen::VectorXd Tools::polar_to_cart(const MeasurementPackage &measurement_pack) {
// void Tools::polar_to_cart(const MeasurementPackage &measurement_pack, float &px, float &py) {
  Eigen::VectorXd vect_cart = VectorXd(4);
  
  float rho = measurement_pack.raw_measurements_[0];
  float phi = measurement_pack.raw_measurements_[1];
  float rho_dot = measurement_pack.raw_measurements_[2];
  
  float px = rho * cos(phi); 
  float py = rho * sin(phi);
  float vx = rho_dot * cos(phi);
  float vy = rho_dot * sin(phi);
  
  vect_cart << px, py, vx, vy;
  
  return vect_cart;
}

Eigen::VectorXd Tools::cartesian_to_polar(const Eigen::VectorXd &x) {
  VectorXd z_pred(3);

  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  // If px2 is zero set it to APPROX_ZERO = 0.0001 
  //     if (fabs(px2) < APPROX_ZERO) {
  //       px2 = APPROX_ZERO;
  //     }

  px = convertNearZero(px);

  //convert cartesion to polar
  float px_p = pow(px, 2);
  float py_p = pow(py, 2);
  float rho = sqrt(px_p + py_p); 

  rho = convertNearZero(rho);

  z_pred[0] = rho;
  z_pred[1] = atan2(py, px);
  z_pred[2] = (px * vx + py * vy) / rho;

  return z_pred;

}

void Tools::print_vec(const Eigen::VectorXd& a) {
   std::cout << "The vector elements are : ";
   
   for(unsigned int i=0; i < a.size(); i++)
      cout << a[i] << ' ';
  
   cout << endl;
}

                   
        