#include "base/atomic_types.h"
#include "checked_value.h"
#include "math/linalg/covariance_matrix_policies.h"
#include "motion/motion_model_cv.h"
#include "trackingLib/env/ego_motion.hpp"                       // IWYU pragma: keep
#include "trackingLib/filter/information_filter.hpp"            // IWYU pragma: keep
#include "trackingLib/filter/kalman_filter.hpp"                 // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_io.h"                  // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp"               // IWYU pragma: keep
#include "trackingLib/observation/position_observation_model.h" // IWYU pragma: keep
#include <string>
#include <tuple>

#include <fstream>
#include <iostream>
#include <random>

namespace math        = tracking::math;
namespace motion      = tracking::motion;
namespace observation = tracking::observation;

auto main(int argc, char** argv) -> int
{
  // Optional structured CSV export (one row per step) for offline visualization; see examples/viz.
  // Console output above is unaffected. Path defaults so a plain `./single_linear_object_tracking`
  // run behaves exactly as before.
  const std::string csvPath = (argc > 1) ? argv[1] : "single_linear_track.csv";
  std::ofstream     csv(csvPath);
  csv << "# motion_model=CV\n";
  csv << "step,t,gt_x,gt_y,z_x,z_y,est_x,est_y,est_vx,est_vy,P_xx,P_xy,P_yy,use_kalman\n";

  // Define a MotionModelCV with UDU-factored covariance matrix
  // State: [X, VX, Y, VY] - 4D state for constant velocity model
  using MM = motion::MotionModelCV<math::FactoredCovarianceMatrixPolicy<float64>>;

  // type aliases
  using value_type            = typename MM::value_type;
  using EgoMotionType         = typename MM::EgoMotionType;
  using KalmanFilterType      = typename MM::KalmanFilterType;
  using InformationFilterType = typename MM::InformationFilterType;

  // Observation model for a direct (x, y) position measurement: h(x) = [X, Y]'
  using PositionObservationType =
      observation::PositionObservationModel<math::FactoredCovarianceMatrixPolicy<float64>, motion::StateDefCV>;

  // Create an Filter instance
  const InformationFilterType informationFilter{};
  const KalmanFilterType      kalmanFilter{};

  // Initialize the object state: starting at x=30, y=10. The velocity is unknown at initialization —
  // there is no measurement providing velocity information — so it is set to 0 rather than a prior guess.
  // The filter recovers the true velocity from the sequence of position measurements.
  // Use the factory method FromLists to create the motion model with initial state
  // clang-format off
  auto motionModel = checkedValue(MM::FromLists(
    {30.0, 0.0, 10.0, 0.0}, // State vector: [X, VX, Y, VY]
    {
      {1e-7, 0.0,  0.0,  0.0 },  // Information matrix Y, diagonal in state order [X, VX, Y, VY]:
      {0.0,  1e-3, 0.0,  0.0 },  //   X, Y  positions -> ~no prior information (1e-7, near-singular).
      {0.0,  0.0,  1e-7, 0.0 },  //   VX, VY velocities -> weak prior (1e-3, std ~30 m/s): the initial
      {0.0,  0.0,  0.0,  1e-3}   //   velocity is unknown but bounded to a plausible range, which bounds
  }));                   //   the startup transient symmetrically on both axes without seeding a value.

  // Create ego motion with zero motion (ego vehicle is not moving)
  // This is the scenario where ego vehicle is stationary
  const EgoMotionType::InertialMotion motionParams{
      .v  = static_cast<value_type>(0.0), // velocity
      .a  = static_cast<value_type>(0.0), // acceleration
      .w  = static_cast<value_type>(0.0), // yaw rate
      .sv = static_cast<value_type>(1e-6), // velocity uncertainty
      .sa = static_cast<value_type>(1e-6), // acceleration uncertainty
      .sw = static_cast<value_type>(1e-6)  // yaw rate uncertainty
  };

  const EgoMotionType::Geometry geometry{
      .width                  = static_cast<value_type>(2.0),
      .length                 = static_cast<value_type>(4.5),
      .height                 = static_cast<value_type>(1.5),
      .distCog2Ego            = static_cast<value_type>(0.0),
      .distFrontAxle2Ego      = static_cast<value_type>(0.0),
      .distFrontAxle2RearAxle = static_cast<value_type>(2.8)
  };
  // clang-format on

  // Simulation parameters
  const auto   dt       = static_cast<value_type>(0.1); // time step of 0.1 seconds
  const sint32 numSteps = 40;                           // simulate 40 time steps (4 seconds total)

  // Ground-truth object: a perfect constant-velocity trajectory the filter is trying to recover.
  // The measurements below are noisy samples of this trajectory.
  auto       gtX  = static_cast<value_type>(30.0);
  auto       gtY  = static_cast<value_type>(10.0);
  const auto gtVx = static_cast<value_type>(10.0);
  const auto gtVy = static_cast<value_type>(2.5);

  // Measurement noise: position sensor with a standard deviation of 1.0 m per axis (R = diag(1.0, 1.0)).
  // Each information-form update adds only H'*inv(R)*H = 1/sigma^2 of information, so together with the
  // very low initial information above and a switch threshold tuned for this noise level (see the
  // determinant() check below), the information matrix still needs a few measurements before it is
  // well-conditioned enough to invert — i.e. the InformationFilter bootstraps for a few steps and only
  // then switches to the KalmanFilter.
  const auto measStd = static_cast<value_type>(1.0);
  // clang-format off
  const auto R = checkedValue(PositionObservationType::MeasurementCovFromList({
    {measStd * measStd, static_cast<value_type>(0.0)},
    {static_cast<value_type>(0.0), measStd * measStd}
  }));
  // clang-format on

  // Fixed-seed RNG so the example produces reproducible output run to run.
  std::mt19937                         rng{42U};
  std::normal_distribution<value_type> measNoise{static_cast<value_type>(0.0), measStd};

  std::cout << "Single Object Tracking Example" << '\n';
  std::cout << "with combined Information and Kalman Filter usage" << '\n';
  std::cout << "=================================================" << '\n';
  std::cout << "Scenario: Diagonal crossing object (x=30,y=10, vx=10, vy=2.5)" << '\n';
  std::cout << "Ego vehicle: Stationary (no motion)" << '\n';
  std::cout << "Filter: InformationFilter (suitable for high initial uncertainty)" << '\n';
  std::cout << "Filter: KalmanFilter (after uncertainty has been initialized)" << '\n';
  std::cout << "Motion Model: Constant Velocity (CV)" << '\n';
  std::cout << "Measurements: noisy (x, y) position, std=" << measStd << " m per axis" << '\n';
  std::cout << '\n';

  std::cout << "Initial State:" << '\n';
  std::cout << "  Position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << '\n';
  std::cout << "  Velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << '\n';
  std::cout << "  Initial Information Matrix (Y):" << '\n';
  std::cout << static_cast<const MM&>(motionModel).getCov()() << '\n';
  std::cout << '\n';

  // FromLists stores the state in Cartesian state space, but the InformationFilter operates on the
  // information vector y = Y*x. Transform the initial state into information space once, up front, so
  // the bootstrap predict/update round-trips (info -> state -> info) start from a consistent state.
  motionModel.convertStateVecIntoInformationSpace();

  std::string filter_cov_str = "  Information Matrix (Y):";
  bool        useKalman      = false;

  // Recover the current Cartesian state estimate and covariance. While the InformationFilter is
  // running, the stored vector/matrix are the information vector y = Y*x and information matrix Y
  // rather than the mean and covariance, so both are recovered on a throwaway copy (x = Y^-1 * y,
  // P = Y^-1) purely for display/export — the running filter state is left untouched.
  auto getEstimate = [&]() {
    value_type ex  = motionModel.getX();
    value_type ey  = motionModel.getY();
    value_type evx = motionModel.getVx();
    value_type evy = motionModel.getVy();
    // NOLINTNEXTLINE(modernize-use-auto) -- Copy the numeric value; auto retains a reference wrapper into a temporary.
    value_type pXX = static_cast<const MM&>(motionModel).getCov()()(0, 0).value();
    // NOLINTNEXTLINE(modernize-use-auto) -- Copy the numeric value; auto retains a reference wrapper into a temporary.
    value_type pXY = static_cast<const MM&>(motionModel).getCov()()(0, 2).value();
    // NOLINTNEXTLINE(modernize-use-auto) -- Copy the numeric value; auto retains a reference wrapper into a temporary.
    value_type pYY = static_cast<const MM&>(motionModel).getCov()()(2, 2).value();
    if (!useKalman)
    {
      MM tmp = static_cast<const MM&>(motionModel);
      tmp.convertStateVecIntoStateSpace();
      ex  = tmp.getX();
      ey  = tmp.getY();
      evx = tmp.getVx();
      evy = tmp.getVy();
      if (tmp.invertCov().has_value())
      {
        pXX = static_cast<const MM&>(tmp).getCov()()(0, 0).value();
        pXY = static_cast<const MM&>(tmp).getCov()()(0, 2).value();
        pYY = static_cast<const MM&>(tmp).getCov()()(2, 2).value();
      }
    }
    return std::make_tuple(ex, ey, evx, evy, pXX, pXY, pYY);
  };

  auto printEstimate = [&](const char* label) {
    const auto [ex, ey, evx, evy, pXX, pXY, pYY] = getEstimate();
    static_cast<void>(pXX);
    static_cast<void>(pXY);
    static_cast<void>(pYY);
    std::cout << label << " (" << ex << ", " << ey << ") v=(" << evx << ", " << evy << ")" << '\n';
  };

  // Simulation loop
  for (sint32 step = 0; step < numSteps; ++step)
  {
    const value_type currentTime = step * dt;

    // Create ego motion for this time step (zero motion)
    auto egoMotion = checkedValue(EgoMotionType::TryCreate(motionParams, geometry, dt));

    std::cout << "Step " << step << " (t=" << currentTime << "s):" << '\n';

    // Advance the ground-truth object by one time step and take a noisy (x, y) position measurement of it.
    gtX += gtVx * dt;
    gtY += gtVy * dt;
    const value_type zx             = gtX + measNoise(rng);
    const value_type zy             = gtY + measNoise(rng);
    const auto       measurementVec = checkedValue(PositionObservationType::MeasurementVecFromList({zx, zy}));
    const auto       obs            = checkedValue(PositionObservationType::TryCreate(measurementVec, R));

    if (useKalman)
    {
      // Kalman regime: predict, then correct with the (x, y) measurement in state space.
      motionModel.predict(dt, kalmanFilter, egoMotion);
      printEstimate("  Predicted State:");
      motionModel.update(kalmanFilter, egoMotion, obs);
    }
    else
    {
      // Bootstrap regime: predict and correct in information space. The measurement update is a purely
      // additive accumulation of information (Y += H'*inv(R)*H), which is what grows the information
      // matrix until its covariance becomes well-defined.
      motionModel.predict(dt, informationFilter, egoMotion);
      printEstimate("  Predicted State:");
      motionModel.update(informationFilter, egoMotion, obs);

      // Hand over to the Kalman filter once the information matrix is well-conditioned. Recover the state
      // mean from the information vector first (needs Y while it is still the information matrix), then
      // invert the information matrix into a covariance (Y -> P). The threshold (1e-2, vs. 1e-6 when
      // measStd was 3.0) is tuned for the current measStd so the bootstrap still spans a few steps.
      if (static_cast<const MM&>(motionModel).getCov().determinant() > 1e-2)
      {
        motionModel.convertStateVecIntoStateSpace();
        if (motionModel.invertCov().has_value())
        {
          std::cout << "  !!! Switching from InformationFilter to KalmanFilter !!!" << '\n';
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

    std::cout << "  Measurement z:   (" << zx << ", " << zy << ")  [ground truth: (" << gtX << ", " << gtY << ")]" << '\n';
    printEstimate("  Corrected State:");

    std::cout << filter_cov_str << '\n';
    std::cout << static_cast<const MM&>(motionModel).getCov()() << '\n';
    std::cout << '\n';

    const auto [ex, ey, evx, evy, pXX, pXY, pYY] = getEstimate();
    csv << step << ',' << currentTime << ',' << gtX << ',' << gtY << ',' << zx << ',' << zy << ',' << ex << ',' << ey << ','
        << evx << ',' << evy << ',' << pXX << ',' << pXY << ',' << pYY << ',' << (static_cast<int>(useKalman)) << '\n';
  }

  std::cout << "Simulation completed successfully!" << '\n';
  std::cout << "Final position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << '\n';
  std::cout << "Final velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << '\n';

  return 0;
}
