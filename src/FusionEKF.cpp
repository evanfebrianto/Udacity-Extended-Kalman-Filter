#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <tuple> 

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


/*
TIP: To run on the simulator, one can navigate to the respective directory and run cmd line "/home/workspace/simulator# ./term2_sim.x86_64" in the terminal 

*/

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  
  noise_ax = 9;
  noise_ay = 9;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
   H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

//   ekf_.P_ = MatrixXd(4,4);
   ekf_.initP_(999); //pass large variance to initialize state covariance matrix  
//   ekf_.F_ = MatrixXd(4,4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
//   float rho, phi, rho_dot;
  float px, py, vx, vy;
//   float px, py;
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      VectorXd vec_cart = VectorXd(4);
      vec_cart = tools.polar_to_cart(measurement_pack);
      tools.print_vec(vec_cart);
      ekf_.setx_(vec_cart);
      
      std::tuple<int, int> v_pair (vec_cart[1],vec_cart[2]);
      std::tie (vx, vy) = v_pair;
      
      //check velocity values for debugging 
      std::cout << "Velocity Pair: ";
      std::cout << std::get<0>(v_pair) << ' ';
      std::cout << std::get<1>(v_pair) << endl;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      //No coordinate transformation required for lidar measurements
        px = measurement_pack.raw_measurements_[0];
        py = measurement_pack.raw_measurements_[1];

        //store the first measurement's position and velocity in the x state vector 
        VectorXd vect_x = VectorXd(4);
        vect_x << px, py, 0, 0;
        ekf_.setx_(vect_x);
    }

    previous_timestamp_ = measurement_pack.timestamp_;
   //Done initializing. No need to predict or update.
    is_initialized_ = true;
    return;
  }

  /**
   * PREDICTION
   */

  /**
   * 1. Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    
  // Compute the time elapsed in seconds between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  //Update state transition matrix F according to the new elapsed time.
  ekf_.setF_(dt);
  
  //Update noise_ax = 9 and noise_ay = 9 for your Q matrix.
  ekf_.setQ_(dt, noise_ax, noise_ay);

  // Predict state vector x and covairance matrix P 
  ekf_.Predict();

  /**
   * UPDATE MEASUREMENT
   */

  /**
   * 2. Update measurement and step depending on sensor type.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.getx_());
    
	ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_, ekf_.H_ );
  } else {
    // TODO: Laser updates
	ekf_.Update(measurement_pack.raw_measurements_, R_laser_, H_laser_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
