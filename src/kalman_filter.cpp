#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const float kVar = 1000;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
	//Initialize variables when invoked from FusionEKF
  x_ = VectorXd(4);

  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Q_ = MatrixXd(4, 4);

  P_ = MatrixXd(4, 4);
//   P_ << kVar, 0, 0, 0,
//         0, kVar, 0, 0,
//         0, 0, kVar, 0,
//         0, 0, 0, kVar;
  
  
}

KalmanFilter::~KalmanFilter() {}

//function consolidated with default constructor
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  
}

//Accessors
Eigen::VectorXd KalmanFilter::getx_() {
  return x_;
}

//Mutators 
void KalmanFilter::setx_(const Eigen::VectorXd x) {
  x_ = x;
}

void KalmanFilter::setF_(float dt) {
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

void KalmanFilter::setQ_(float dt, float noise_ax, float noise_ay) {
  float dt2 = pow(dt, 2);
  float dt3 = pow(dt, 3);
  float dt4 = pow(dt, 4);

  Q_ << noise_ax * dt4/4, 0, noise_ax*dt3/2, 0,
        0, noise_ay*dt4/4, 0, noise_ay*dt3/2,
        noise_ax * dt3/2, 0, noise_ax*dt2, 0,
        0, noise_ay*dt3/2, 0, noise_ay*dt2;
}

void KalmanFilter::initP_(float variance) {

  P_ << variance, 0, 0, 0,
			       0, variance, 0, 0,
			       0, 0, variance, 0,
			       0, 0, 0, variance;
}


void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  	//predict state using state transition matrix and earlier state
	x_ = F_ * x_;
	//Transpose of state transition matrix
	MatrixXd Ft = F_.transpose();
	//covariance matrix , Q is process noise mtx
	P_ = F_ * P_ * Ft + Q_;
}

//Lidar Kalman Filter Update 
void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &H) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
	R_ = R;
  	H_ = H; //measurement matrix 
  
  	//both standard (linear) and extended KF 
	VectorXd z_pred = H_ * x_;
	
  	//Update measurements KF 
	ApplyKF(z, z_pred);

	
}

//Radar Extended Kalman Filter Update
void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &Hj) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  R_ = R;
  H_ = Hj; //Jacobian
  //convert Cartesian coords of current state to polar coords to compare against Radar measurement. Returns prediction vector. 
  VectorXd z_pred = tools.cartesian_to_polar(x_);

  	//
  ApplyKF(z, z_pred);
	
}

void KalmanFilter::ApplyKF(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred) {
  // Perform various matrix calculations
  
  //evaluate error between predicted & measured state values
  VectorXd y = z - z_pred;
  
  // normalize ϕ in the y vector so that its angle is between −π and π
  while (y(1) < -M_PI) y(1) += 2*M_PI;
  while (y(1) > M_PI) y(1) -= 2*M_PI;
  
  //Update state and covariance matrices with Kalman gain K 
  MatrixXd Ht = H_.transpose();     // H_ contains the Jacobian (for radar) or H matrix (for lidar)
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New estimates for x and P
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
