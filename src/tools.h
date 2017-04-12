#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

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
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to convert from polar to cartesian.
  */
  Eigen::VectorXd ConvertPolarToCartesian(const Eigen::VectorXd& measurements);

  /**
  * Normalize angles
  */
  double NormalizeAngle(double phi);
};

#endif /* TOOLS_H_ */
