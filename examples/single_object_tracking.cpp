#include "math/linalg/covariance_matrix_policies.h"
#include "trackingLib/env/ego_motion.hpp"                       // IWYU pragma: keep
#include "trackingLib/filter/information_filter.hpp"            // IWYU pragma: keep
#include "trackingLib/filter/kalman_filter.hpp"                 // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_io.h"                  // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp"               // IWYU pragma: keep
#include "trackingLib/observation/position_observation_model.h" // IWYU pragma: keep

#include <iostream>
#include <random>

using namespace tracking;

int main()
{
  // Define a MotionModelCV with full covariance matrix
  // State: [X, VX, Y, VY] - 4D state for constant velocity model
  using MM = motion::MotionModelCV<math::FullCovarianceMatrixPolicy<float64>>;

  // type aliases
  using value_type            = typename MM::value_type;
  using EgoMotionType         = typename MM::EgoMotionType;
  using KalmanFilterType      = typename MM::KalmanFilterType;
  using InformationFilterType = typename MM::InformationFilterType;

  // Observation model for a direct (x, y) position measurement: h(x) = [X, Y]'
  using PositionObservationType =
      observation::PositionObservationModel<math::FullCovarianceMatrixPolicy<float64>, motion::StateDefCV>;

  // Create an Filter instance
  InformationFilterType informationFilter{};
  KalmanFilterType      kalmanFilter{};

  // Initialize the object state: starting at x=30, y=10. The velocity is unknown at initialization —
  // there is no measurement providing velocity information — so it is set to 0 rather than a prior guess.
  // The filter recovers the true velocity from the sequence of position measurements.
  // Use the factory method FromLists to create the motion model with initial state
  // clang-format off
  auto motionModel = MM::FromLists(
    {30.0, 0.0, 10.0, 0.0}, // State vector: [X, VX, Y, VY]
    {
      {1e-7, 0.0,  0.0,  0.0 },  // Information matrix Y, diagonal in state order [X, VX, Y, VY]:
      {0.0,  1e-3, 0.0,  0.0 },  //   X, Y  positions -> ~no prior information (1e-7, near-singular).
      {0.0,  0.0,  1e-7, 0.0 },  //   VX, VY velocities -> weak prior (1e-3, std ~30 m/s): the initial
      {0.0,  0.0,  0.0,  1e-3}   //   velocity is unknown but bounded to a plausible range, which bounds
  });                           //   the startup transient symmetrically on both axes without seeding a value.

  // Create ego motion with zero motion (ego vehicle is not moving)
  // This is the scenario where ego vehicle is stationary
  EgoMotionType::InertialMotion motionParams{
      .v  = static_cast<value_type>(0.0), // velocity
      .a  = static_cast<value_type>(0.0), // acceleration
      .w  = static_cast<value_type>(0.0), // yaw rate
      .sv = static_cast<value_type>(1e-6), // velocity uncertainty
      .sa = static_cast<value_type>(1e-6), // acceleration uncertainty
      .sw = static_cast<value_type>(1e-6)  // yaw rate uncertainty
  };

  EgoMotionType::Geometry geometry{
      .width                  = static_cast<value_type>(2.0),
      .length                 = static_cast<value_type>(4.5),
      .height                 = static_cast<value_type>(1.5),
      .distCog2Ego            = static_cast<value_type>(0.0),
      .distFrontAxle2Ego      = static_cast<value_type>(0.0),
      .distFrontAxle2RearAxle = static_cast<value_type>(2.8)
  };
  // clang-format on

  // Simulation parameters
  const value_type dt       = static_cast<value_type>(0.1); // time step of 0.1 seconds
  const sint32     numSteps = 20;                           // simulate 20 time steps (2 seconds total)

  // Ground-truth object: a perfect constant-velocity trajectory the filter is trying to recover.
  // The measurements below are noisy samples of this trajectory.
  value_type       gtX  = static_cast<value_type>(30.0);
  value_type       gtY  = static_cast<value_type>(10.0);
  const value_type gtVx = static_cast<value_type>(10.0);
  const value_type gtVy = static_cast<value_type>(2.5);

  // Measurement noise: position sensor with a standard deviation of 3.0 m per axis (R = diag(9.0, 9.0)).
  // The noise is deliberately large: each information-form update adds only H'*inv(R)*H = 1/sigma^2 of
  // information, so together with the very low initial information above the information matrix needs a
  // few measurements before its determinant is large enough to invert — i.e. the InformationFilter
  // bootstraps for three steps and only then switches to the KalmanFilter.
  const value_type measStd = static_cast<value_type>(3.0);
  // clang-format off
  const PositionObservationType::MeasurementCov R = PositionObservationType::MeasurementCovFromList({
    {measStd * measStd, static_cast<value_type>(0.0)},
    {static_cast<value_type>(0.0), measStd * measStd}
  });
  // clang-format on

  // Fixed-seed RNG so the example produces reproducible output run to run.
  std::mt19937                         rng{42U};
  std::normal_distribution<value_type> measNoise{static_cast<value_type>(0.0), measStd};

  std::cout << "Single Object Tracking Example" << std::endl;
  std::cout << "with combined Information and Kalman Filter usage" << std::endl;
  std::cout << "=================================================" << std::endl;
  std::cout << "Scenario: Diagonal crossing object (x=30,y=10, vx=10, vy=2.5)" << std::endl;
  std::cout << "Ego vehicle: Stationary (no motion)" << std::endl;
  std::cout << "Filter: InformationFilter (suitable for high initial uncertainty)" << std::endl;
  std::cout << "Filter: KalmanFilter (after uncertainty has been initialized)" << std::endl;
  std::cout << "Motion Model: Constant Velocity (CV)" << std::endl;
  std::cout << "Measurements: noisy (x, y) position, std=" << measStd << " m per axis" << std::endl;
  std::cout << std::endl;

  std::cout << "Initial State:" << std::endl;
  std::cout << "  Position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << std::endl;
  std::cout << "  Velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;
  std::cout << "  Initial Information Matrix (Y):" << std::endl;
  std::cout << static_cast<const MM&>(motionModel).getCov()() << std::endl;
  std::cout << std::endl;

  // FromLists stores the state in Cartesian state space, but the InformationFilter operates on the
  // information vector y = Y*x. Transform the initial state into information space once, up front, so
  // the bootstrap predict/update round-trips (info -> state -> info) start from a consistent state.
  motionModel.convertStateVecIntoInformationSpace();

  std::string filter_cov_str = "  Information Matrix (Y):";
  bool        useKalman      = false;

  // Print the current Cartesian state estimate. While the InformationFilter is running, the stored vector
  // is the information vector y = Y*x rather than the mean, so the mean is recovered on a throwaway copy
  // (x = Y^-1 * y) purely for display — the running filter state is left untouched.
  auto printEstimate = [&](const char* label) {
    value_type ex  = motionModel.getX();
    value_type ey  = motionModel.getY();
    value_type evx = motionModel.getVx();
    value_type evy = motionModel.getVy();
    if (!useKalman)
    {
      MM tmp = static_cast<const MM&>(motionModel);
      tmp.convertStateVecIntoStateSpace();
      ex  = tmp.getX();
      ey  = tmp.getY();
      evx = tmp.getVx();
      evy = tmp.getVy();
    }
    std::cout << label << " (" << ex << ", " << ey << ") v=(" << evx << ", " << evy << ")" << std::endl;
  };

  // Simulation loop
  for (sint32 step = 0; step < numSteps; ++step)
  {
    const value_type currentTime = step * dt;

    // Create ego motion for this time step (zero motion)
    EgoMotionType egoMotion(motionParams, geometry, dt);

    std::cout << "Step " << step << " (t=" << currentTime << "s):" << std::endl;

    // Advance the ground-truth object by one time step and take a noisy (x, y) position measurement of it.
    gtX += gtVx * dt;
    gtY += gtVy * dt;
    const value_type              zx = gtX + measNoise(rng);
    const value_type              zy = gtY + measNoise(rng);
    const PositionObservationType obs{PositionObservationType::MeasurementVec::FromList({zx, zy}), R};

    if (useKalman)
    {
      // Kalman regime: predict, then correct with the (x, y) measurement in state space.
      motionModel.predict(dt, kalmanFilter, egoMotion);
      printEstimate("  Predicted State:");
      motionModel.update(kalmanFilter, obs);
    }
    else
    {
      // Bootstrap regime: predict and correct in information space. The measurement update is a purely
      // additive accumulation of information (Y += H'*inv(R)*H), which is what grows the information
      // matrix until its covariance becomes well-defined.
      motionModel.predict(dt, informationFilter, egoMotion);
      printEstimate("  Predicted State:");
      motionModel.update(informationFilter, obs);

      // Hand over to the Kalman filter once the information matrix is well-conditioned. Recover the state
      // mean from the information vector first (needs Y while it is still the information matrix), then
      // invert the information matrix into a covariance (Y -> P).
      if (static_cast<const MM&>(motionModel).getCov().determinant() > 1e-6)
      {
        motionModel.convertStateVecIntoStateSpace();
        if (motionModel.invertCov().has_value())
        {
          std::cout << "  !!! Switching from InformationFilter to KalmanFilter !!!" << std::endl;
          filter_cov_str = "  Covariance Matrix (P):";
          useKalman      = true;
        }
        else
        {
          // Defensive: inversion unexpectedly failed; restore the information-space vector and keep bootstrapping.
          motionModel.convertStateVecIntoInformationSpace();
        }
      }
    }

    std::cout << "  Measurement z:   (" << zx << ", " << zy << ")  [ground truth: (" << gtX << ", " << gtY << ")]" << std::endl;
    printEstimate("  Corrected State:");

    std::cout << filter_cov_str << std::endl;
    std::cout << static_cast<const MM&>(motionModel).getCov()() << std::endl;
    std::cout << std::endl;
  }

  std::cout << "Simulation completed successfully!" << std::endl;
  std::cout << "Final position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << std::endl;
  std::cout << "Final velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;

  return 0;
}