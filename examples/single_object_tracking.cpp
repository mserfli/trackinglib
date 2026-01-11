#include "trackingLib/env/ego_motion.hpp"
#include "trackingLib/filter/information_filter.hpp"
#include "trackingLib/math/linalg/matrix_io.h"
#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep

#include <iomanip>
#include <iostream>

using namespace tracking;

#define CovarianceMatrixType math::CovarianceMatrixFull
using FloatType = float64;

int main()
{
  // Create an InformationFilter instance
  filter::InformationFilter<FloatType> informationFilter;

  // Create a MotionModelCV with full covariance matrix
  // State: [X, VX, Y, VY] - 4D state for constant velocity model
  using MM = motion::MotionModelCV<CovarianceMatrixType, FloatType>;

  // Initialize the object state: starting at x=30, y=10 with vx=10, vy=2.5
  // This represents a diagonal crossing object
  // Use the factory method FromLists to create the motion model with initial state
  // clang-format off
  auto motionModel = MM::FromLists(
    {30.0, 10.0, 10.0, 2.5}, // State vector: [X, VX, Y, VY]
    {
      {0.01, 0.0, 0.0, 0.0},  // Information matrix Y (low confidence)
      {0.0, 0.01, 0.0, 0.0},
      {0.0, 0.0, 0.01, 0.0},
      {0.0, 0.0, 0.0, 0.01}
  });
  // clang-format off

  // Create ego motion with zero motion (ego vehicle is not moving)
  // This is the scenario where ego vehicle is stationary
  env::EgoMotion<FloatType>::InertialMotion motionParams{
      .v  = static_cast<FloatType>(0.0), // velocity
      .a  = static_cast<FloatType>(0.0), // acceleration
      .w  = static_cast<FloatType>(0.0), // yaw rate
      .sv = static_cast<FloatType>(0.0), // velocity uncertainty
      .sa = static_cast<FloatType>(0.0), // acceleration uncertainty
      .sw = static_cast<FloatType>(0.0)  // yaw rate uncertainty
  };

  env::EgoMotion<FloatType>::Geometry geometry{.width                  = static_cast<FloatType>(2.0),
                                               .length                 = static_cast<FloatType>(4.5),
                                               .height                 = static_cast<FloatType>(1.5),
                                               .distCog2Ego            = static_cast<FloatType>(0.0),
                                               .distFrontAxle2Ego      = static_cast<FloatType>(0.0),
                                               .distFrontAxle2RearAxle = static_cast<FloatType>(2.8)};

  // Simulation parameters
  const FloatType dt       = static_cast<FloatType>(0.1); // time step of 0.1 seconds
  const sint32    numSteps = 20;                          // simulate 20 time steps (2 seconds total)

  std::cout << "Single Object Tracking Example with InformationFilter" << std::endl;
  std::cout << "=====================================================" << std::endl;
  std::cout << "Scenario: Diagonal crossing object (x=30,y=10, vx=10, vy=2.5)" << std::endl;
  std::cout << "Ego vehicle: Stationary (no motion)" << std::endl;
  std::cout << "Filter: InformationFilter (suitable for high initial uncertainty)" << std::endl;
  std::cout << "Motion Model: Constant Velocity (CV)" << std::endl;
  std::cout << std::endl;

  std::cout << "Initial State:" << std::endl;
  std::cout << "  Position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << std::endl;
  std::cout << "  Velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;
  std::cout << "  Initial Information Matrix (Y):" << std::endl;
  std::cout << static_cast<const MM&>(motionModel).getCov() << std::endl;
  std::cout << std::endl;

  // Simulation loop
  for (sint32 step = 0; step < numSteps; ++step)
  {
    const FloatType currentTime = step * dt;

    // Create ego motion for this time step (zero motion)
    env::EgoMotion<FloatType> egoMotion(motionParams, geometry, dt);

    std::cout << "Step " << step << " (t=" << currentTime << "s):" << std::endl;
    std::cout << "  Current State: (" << motionModel.getX() << ", " << motionModel.getY() << ") " << "v=(" << motionModel.getVx()
              << ", " << motionModel.getVy() << ")" << std::endl;

    // Predict the next state using InformationFilter
    motionModel.predict(dt, informationFilter, egoMotion);

    std::cout << "  Predicted State: (" << motionModel.getX() << ", " << motionModel.getY() << ") " << "v=("
              << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;

    std::cout << "  Information Matrix (Y):" << std::endl;
    std::cout << static_cast<const MM&>(motionModel).getCov() << std::endl;
    std::cout << std::endl;
  }

  std::cout << "Simulation completed successfully!" << std::endl;
  std::cout << "Final position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << std::endl;
  std::cout << "Final velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;

  return 0;
}