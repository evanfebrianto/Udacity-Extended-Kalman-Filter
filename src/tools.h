#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

const float NEAR_ZERO = 0.0001; //this will overide x or y if one is zero 

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
   * A helper method to convert polar to cartesian coordinates.
   */
  Eigen::VectorXd polar_to_cart(const MeasurementPackage &measurement_pack);
  
  
  /**
   * A helper method to convert cartesion to polar coordinates.
   */
  Eigen::VectorXd cartesian_to_polar(const Eigen::VectorXd &x);
  
  /**
   * A helper method to check and modifiy if a divisor is zero. 
   */
  float convertNearZero(float &state_val);
  
  void print_vec(const Eigen::VectorXd& a);
  
};

#endif  // TOOLS_H_
