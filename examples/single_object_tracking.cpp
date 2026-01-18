#include "math/linalg/covariance_matrix_policies.h"
#include "trackingLib/env/ego_motion.hpp"            // IWYU pragma: keep
#include "trackingLib/filter/information_filter.hpp" // IWYU pragma: keep
#include "trackingLib/filter/kalman_filter.hpp"      // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_io.h"       // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp"    // IWYU pragma: keep

#include <iostream>

using namespace tracking;

int main()
{
  // Define a MotionModelCV with full covariance matrix
  // State: [X, VX, Y, VY] - 4D state for constant velocity model
  using MM = motion::MotionModelCV<math::FullCovarianceMatrixPolicy<float64>>;

  // type aliases
  using FloatType             = typename MM::FloatType;
  using EgoMotionType         = typename MM::EgoMotionType;
  using KalmanFilterType      = typename MM::KalmanFilterType;
  using InformationFilterType = typename MM::InformationFilterType;

  // Create an Filter instance
  InformationFilterType informationFilter{};
  KalmanFilterType      kalmanFilter{};

  // Initialize the object state: starting at x=30, y=10 with vx=10, vy=2.5
  // This represents a diagonal crossing object
  // Use the factory method FromLists to create the motion model with initial state
  // clang-format off
  auto motionModel = MM::FromLists(
    {30.0, 10.0, 10.0, 2.5}, // State vector: [X, VX, Y, VY]
    {
      {0.001, 0.0, 0.0, 0.0},  // Information matrix Y (low confidence)
      {0.0, 0.001, 0.0, 0.0},
      {0.0, 0.0, 0.001, 0.0},
      {0.0, 0.0, 0.0, 0.001}
  });

  // Create ego motion with zero motion (ego vehicle is not moving)
  // This is the scenario where ego vehicle is stationary
  EgoMotionType::InertialMotion motionParams{
      .v  = static_cast<FloatType>(0.0), // velocity
      .a  = static_cast<FloatType>(0.0), // acceleration
      .w  = static_cast<FloatType>(0.0), // yaw rate
      .sv = static_cast<FloatType>(1e-6), // velocity uncertainty
      .sa = static_cast<FloatType>(1e-6), // acceleration uncertainty
      .sw = static_cast<FloatType>(1e-6)  // yaw rate uncertainty
  };

  EgoMotionType::Geometry geometry{
      .width                  = static_cast<FloatType>(2.0),
      .length                 = static_cast<FloatType>(4.5),
      .height                 = static_cast<FloatType>(1.5),
      .distCog2Ego            = static_cast<FloatType>(0.0),
      .distFrontAxle2Ego      = static_cast<FloatType>(0.0),
      .distFrontAxle2RearAxle = static_cast<FloatType>(2.8)
  };
  // clang-format on

  // Simulation parameters
  const FloatType dt       = static_cast<FloatType>(0.1); // time step of 0.1 seconds
  const sint32    numSteps = 20;                          // simulate 20 time steps (2 seconds total)

  std::cout << "Single Object Tracking Example" << std::endl;
  std::cout << "with combined Information and Kalman Filter usage" << std::endl;
  std::cout << "=================================================" << std::endl;
  std::cout << "Scenario: Diagonal crossing object (x=30,y=10, vx=10, vy=2.5)" << std::endl;
  std::cout << "Ego vehicle: Stationary (no motion)" << std::endl;
  std::cout << "Filter: InformationFilter (suitable for high initial uncertainty)" << std::endl;
  std::cout << "Filter: KalmanFilter (after uncertainty has been initialized)" << std::endl;
  std::cout << "Motion Model: Constant Velocity (CV)" << std::endl;
  std::cout << std::endl;

  std::cout << "Initial State:" << std::endl;
  std::cout << "  Position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << std::endl;
  std::cout << "  Velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;
  std::cout << "  Initial Information Matrix (Y):" << std::endl;
  std::cout << static_cast<const MM&>(motionModel).getCov()() << std::endl;
  std::cout << std::endl;

  std::string filter_cov_str = "  Information Matrix (Y):";
  bool        useKalman      = false;
  // Simulation loop
  for (sint32 step = 0; step < numSteps; ++step)
  {
    const FloatType currentTime = step * dt;

    // Create ego motion for this time step (zero motion)
    EgoMotionType egoMotion(motionParams, geometry, dt);

    std::cout << "Step " << step << " (t=" << currentTime << "s):" << std::endl;

    if (useKalman)
    {
      motionModel.predict(dt, kalmanFilter, egoMotion);
    }
    else
    {
      // accumulate Information
      motionModel.predict(dt, informationFilter, egoMotion);

      if ((static_cast<const MM&>(motionModel).getCov().determinant() > 1e-6) && (motionModel.invertCov().has_value()))
      {
        // volume of information matrix was big enough and covariance transformation succeeded
        std::cout << "!!! Switching from InformationFilter to KalmanFilter !!!\n" << std::endl;
        filter_cov_str = "  Covariance Matrix (P):";
        useKalman      = true;
      }
    }

    std::cout << "  Predicted State: (" << motionModel.getX() << ", " << motionModel.getY() << ") " << "v=("
              << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;

    std::cout << filter_cov_str << std::endl;
    std::cout << static_cast<const MM&>(motionModel).getCov()() << std::endl;
    std::cout << std::endl;
  }

  std::cout << "Simulation completed successfully!" << std::endl;
  std::cout << "Final position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << std::endl;
  std::cout << "Final velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;

  return 0;
}