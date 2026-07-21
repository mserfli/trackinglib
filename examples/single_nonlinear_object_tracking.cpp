#include "math/linalg/covariance_matrix_policies.h"
#include "trackingLib/env/ego_motion.hpp"                                    // IWYU pragma: keep
#include "trackingLib/filter/information_filter.hpp"                         // IWYU pragma: keep
#include "trackingLib/filter/kalman_filter.hpp"                              // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_io.h"                               // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp"                            // IWYU pragma: keep
#include "trackingLib/observation/range_bearing_doppler_observation_model.h" // IWYU pragma: keep
#include "trackingLib/observation/sensor_mounting_pose.h"                    // IWYU pragma: keep

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

using namespace tracking;

int main(int argc, char** argv)
{
  // Optional structured CSV export (one row per step) for offline visualization; see examples/viz.
  // Console output above is unaffected. Path defaults so a plain `./single_nonlinear_object_tracking`
  // run behaves exactly as before.
  const std::string csvPath = (argc > 1) ? argv[1] : "single_nonlinear_track.csv";
  std::ofstream     csv(csvPath);
  csv << "# motion_model=CV\n";
  csv << "step,t,gt_x,gt_y,est_x,est_y,est_vx,est_vy,P_xx,P_xy,P_yy,use_kalman,"
      << "ego_world_x,ego_world_y,ego_world_psi,target_world_x,target_world_y," << "z_range,z_bearing,z_doppler\n";

  // Define a MotionModelCV with UDU-factored covariance matrix
  // State: [X, VX, Y, VY] - 4D state for constant velocity model
  using MM = motion::MotionModelCV<math::FactoredCovarianceMatrixPolicy<float64>>;

  // type aliases
  using value_type            = typename MM::value_type;
  using EgoMotionType         = typename MM::EgoMotionType;
  using KalmanFilterType      = typename MM::KalmanFilterType;
  using InformationFilterType = typename MM::InformationFilterType;

  // Nonlinear observation model measuring polar range/bearing/doppler, e.g. a radar
  using RangeBearingDopplerType =
      observation::RangeBearingDopplerObservationModel<math::FactoredCovarianceMatrixPolicy<float64>, motion::StateDefCV>;

  InformationFilterType informationFilter{};
  KalmanFilterType      kalmanFilter{};

  // Initialize the object state: starting at x=60, y=-20 relative to the tracking frame, moving
  // roughly towards the ego vehicle's path. The velocity is unknown at initialization - a single
  // radar detection gives no velocity information - so it is set to 0 rather than a prior guess.
  // clang-format off
  auto motionModel = MM::FromLists(
    {60.0, 0.0, -20.0, 0.0}, // State vector: [X, VX, Y, VY]
    {
      {1e-9, 0.0,  0.0,  0.0 },  // Information matrix Y, diagonal in state order [X, VX, Y, VY]:
      {0.0,  1e-5, 0.0,  0.0 },  //   X, Y  positions -> ~no prior information (1e-9, near-singular).
      {0.0,  0.0,  1e-9, 0.0 },  //   VX, VY velocities -> weak prior (1e-5, std ~300 m/s): the initial
      {0.0,  0.0,  0.0,  1e-5}   //   velocity is unknown but bounded to a plausible range, which bounds
  });                           //   the startup transient symmetrically on both axes without seeding a value.

  // Ego vehicle drives a gentle left turn: constant forward speed with a non-zero yaw rate.
  EgoMotionType::InertialMotion motionParams{
      .v  = static_cast<value_type>(15.0),  // velocity [m/s]
      .a  = static_cast<value_type>(0.0),   // acceleration
      .w  = static_cast<value_type>(0.05),  // yaw rate [rad/s]
      .sv = static_cast<value_type>(0.1),   // velocity uncertainty
      .sa = static_cast<value_type>(1e-3),  // acceleration uncertainty
      .sw = static_cast<value_type>(0.01)   // yaw rate uncertainty
  };

  EgoMotionType::Geometry geometry{
      .width                  = static_cast<value_type>(2.0),
      .length                 = static_cast<value_type>(4.5),
      .height                 = static_cast<value_type>(1.5),
      .distCog2Ego            = static_cast<value_type>(0.0),
      .distFrontAxle2Ego      = static_cast<value_type>(0.0),
      .distFrontAxle2RearAxle = static_cast<value_type>(2.8)
  };

  // The radar is front-mounted, 2 m ahead and 0.2 m to the right of the tracking-frame origin,
  // with a slight -0.1 deg mounting yaw offset relative to the tracking-frame heading.
  const RangeBearingDopplerType::BaseExtendedObservationModel::SensorPose sensorPose =
      RangeBearingDopplerType::BaseExtendedObservationModel::SensorPose::FromValues(
          static_cast<value_type>(2.0), static_cast<value_type>(-0.2),
          static_cast<value_type>(-0.1 * M_PI / 180.0));
  // clang-format on

  // Simulation parameters
  const value_type dt       = static_cast<value_type>(0.1); // time step of 0.1 seconds
  const sint32     numSteps = 30;                           // simulate 30 time steps (3 seconds total)

  // Ground truth is easiest to reason about in a fixed world frame: the ego vehicle drives its
  // CTRV arc, the target drives its own constant-velocity line, and both are independent of each
  // other. What the filter actually tracks is the target expressed in the tracking frame, which is
  // ego-centered and ego-aligned - so every step the world-frame ground truth is re-expressed
  // relative to the ego's new world pose before it is used to synthesize a measurement. This is
  // exactly the transform predict() performs internally via ego-motion compensation.
  value_type egoWorldX   = static_cast<value_type>(0.0);
  value_type egoWorldY   = static_cast<value_type>(0.0);
  value_type egoWorldPsi = static_cast<value_type>(0.0);

  // Target world-frame state: starts at the initial tracking-frame position (tracking frame and
  // world frame coincide at t=0, since the ego starts at the world origin with zero heading).
  value_type       targetWorldX = static_cast<value_type>(60.0);
  value_type       targetWorldY = static_cast<value_type>(-20.0);
  const value_type targetVx     = static_cast<value_type>(-8.0);
  const value_type targetVy     = static_cast<value_type>(3.0);

  // Measurement noise for the radar: range std 1.0 m, bearing std 1 deg, doppler std 0.5 m/s.
  const value_type rangeStd   = static_cast<value_type>(1.0);
  const value_type bearingStd = static_cast<value_type>(1.0 * M_PI / 180.0);
  const value_type dopplerStd = static_cast<value_type>(0.5);
  // clang-format off
  const RangeBearingDopplerType::MeasurementCov R = RangeBearingDopplerType::MeasurementCovFromList({
    {rangeStd * rangeStd, static_cast<value_type>(0.0),         static_cast<value_type>(0.0)        },
    {static_cast<value_type>(0.0),         bearingStd * bearingStd, static_cast<value_type>(0.0)        },
    {static_cast<value_type>(0.0),         static_cast<value_type>(0.0),         dopplerStd * dopplerStd}
  });
  // clang-format on

  // Fixed-seed RNG so the example produces reproducible output run to run.
  std::mt19937                         rng{42U};
  std::normal_distribution<value_type> rangeNoise{static_cast<value_type>(0.0), rangeStd};
  std::normal_distribution<value_type> bearingNoise{static_cast<value_type>(0.0), bearingStd};
  std::normal_distribution<value_type> dopplerNoise{static_cast<value_type>(0.0), dopplerStd};

  std::cout << "Single Nonlinear Object Tracking Example" << std::endl;
  std::cout << "with RangeBearingDoppler observations and a moving ego vehicle" << std::endl;
  std::cout << "================================================================" << std::endl;
  std::cout << "Scenario: Crossing object (x=60, y=-20, vx=-8, vy=3)" << std::endl;
  std::cout << "Ego vehicle: Moving forward at 15 m/s, turning left at 0.05 rad/s" << std::endl;
  std::cout << "Sensor: front-mounted radar, offset (tx=2 m, ty=-0.2 m, yaw=-0.1 deg) from the tracking-frame origin"
            << std::endl;
  std::cout << "Filter: InformationFilter (suitable for high initial uncertainty)" << std::endl;
  std::cout << "Filter: KalmanFilter (EKF, after uncertainty has been initialized)" << std::endl;
  std::cout << "Motion Model: Constant Velocity (CV)" << std::endl;
  std::cout << "Measurements: noisy (range, bearing, doppler), std=(" << rangeStd << " m, " << (bearingStd * 180.0 / M_PI)
            << " deg, " << dopplerStd << " m/s)" << std::endl;
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

  // Recover the current Cartesian state estimate and covariance. While the InformationFilter is
  // running, the stored vector/matrix are the information vector y = Y*x and information matrix Y
  // rather than the mean and covariance, so both are recovered on a throwaway copy (x = Y^-1 * y,
  // P = Y^-1) purely for display/export - the running filter state is left untouched.
  auto getEstimate = [&]() {
    value_type ex  = motionModel.getX();
    value_type ey  = motionModel.getY();
    value_type evx = motionModel.getVx();
    value_type evy = motionModel.getVy();
    value_type pXX = static_cast<const MM&>(motionModel).getCov()()(0, 0).value();
    value_type pXY = static_cast<const MM&>(motionModel).getCov()()(0, 2).value();
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
    std::cout << label << " (" << ex << ", " << ey << ") v=(" << evx << ", " << evy << ")" << std::endl;
  };

  // Simulation loop
  for (sint32 step = 0; step < numSteps; ++step)
  {
    const value_type currentTime = step * dt;

    // Create ego motion for this time step (constant turn-rate-and-velocity motion)
    EgoMotionType egoMotion(motionParams, geometry, dt);

    std::cout << "Step " << step << " (t=" << currentTime << "s):" << std::endl;

    // Advance the ego vehicle's world-frame pose along its CTRV arc, and the target along its
    // independent constant-velocity world-frame line.
    const value_type psiNew = egoWorldPsi + (motionParams.w * dt);
    if (std::abs(motionParams.w) > static_cast<value_type>(1e-9))
    {
      egoWorldX += (motionParams.v / motionParams.w) * (std::sin(psiNew) - std::sin(egoWorldPsi));
      egoWorldY += (motionParams.v / motionParams.w) * (std::cos(egoWorldPsi) - std::cos(psiNew));
    }
    else
    {
      egoWorldX += motionParams.v * std::cos(egoWorldPsi) * dt;
      egoWorldY += motionParams.v * std::sin(egoWorldPsi) * dt;
    }
    egoWorldPsi = psiNew;

    targetWorldX += targetVx * dt;
    targetWorldY += targetVy * dt;

    // Predict step: motion model advances the state and compensates for the ego vehicle's own
    // motion between the previous and current tracking frame. Bootstrap regime predicts in
    // information space; Kalman regime predicts in state space.
    if (useKalman)
    {
      motionModel.predict(dt, kalmanFilter, egoMotion);
    }
    else
    {
      motionModel.predict(dt, informationFilter, egoMotion);
    }
    printEstimate("  Predicted State:");

    // Re-express the target's world-frame ground truth in the new (ego-centered, ego-aligned)
    // tracking frame: rotate the world-frame offset by the inverse ego heading.
    const value_type cosPsi       = std::cos(egoWorldPsi);
    const value_type sinPsi       = std::sin(egoWorldPsi);
    const value_type worldDx      = targetWorldX - egoWorldX;
    const value_type worldDy      = targetWorldY - egoWorldY;
    const value_type gtX          = (cosPsi * worldDx) + (sinPsi * worldDy);
    const value_type gtY          = (-sinPsi * worldDx) + (cosPsi * worldDy);
    const value_type egoVelWorldX = motionParams.v * std::cos(egoWorldPsi);
    const value_type egoVelWorldY = motionParams.v * std::sin(egoWorldPsi);
    const value_type worldDvx     = targetVx - egoVelWorldX;
    const value_type worldDvy     = targetVy - egoVelWorldY;
    const value_type gtVx         = (cosPsi * worldDvx) + (sinPsi * worldDvy);
    const value_type gtVy         = (-sinPsi * worldDvx) + (cosPsi * worldDvy);

    // Take a noisy radar measurement of the ground-truth target, expressed in the sensor frame.
    // Mirror SensorMountingPose::positionToSensorFrame / directionToSensorFrame: translate by the
    // mounting offset, then rotate by the inverse mounting yaw.
    const value_type mountDx   = gtX - sensorPose.tx();
    const value_type mountDy   = gtY - sensorPose.ty();
    const value_type mountCos  = sensorPose.cosYaw();
    const value_type mountSin  = sensorPose.sinYaw();
    const value_type sensorDx  = (mountCos * mountDx) + (mountSin * mountDy);
    const value_type sensorDy  = (-mountSin * mountDx) + (mountCos * mountDy);
    const value_type sensorDvx = (mountCos * gtVx) + (mountSin * gtVy);
    const value_type sensorDvy = (-mountSin * gtVx) + (mountCos * gtVy);
    const value_type gtRange   = std::sqrt((sensorDx * sensorDx) + (sensorDy * sensorDy));
    const value_type gtBearing = std::atan2(sensorDy, sensorDx);
    const value_type gtDoppler = ((sensorDx * sensorDvx) + (sensorDy * sensorDvy)) / gtRange;

    const value_type zRange   = gtRange + rangeNoise(rng);
    const value_type zBearing = gtBearing + bearingNoise(rng);
    const value_type zDoppler = gtDoppler + dopplerNoise(rng);

    const RangeBearingDopplerType obs{
        RangeBearingDopplerType::MeasurementVec::FromList({zRange, zBearing, zDoppler}), R, sensorPose};

    // Correct step: nonlinear measurement update via the EKF, linearizing h(x) at the current
    // estimate. Bootstrap regime accumulates information (Y += H'*inv(R)*H); Kalman regime corrects
    // the state mean and covariance directly.
    if (useKalman)
    {
      motionModel.update(kalmanFilter, egoMotion, obs);
    }
    else
    {
      motionModel.update(informationFilter, egoMotion, obs);

      // Hand over to the Kalman filter once the information matrix is well-conditioned. The threshold
      // is picked high enough that a single radar detection - which is already fairly informative on
      // range/bearing - does not trigger an immediate switch, so the InformationFilter bootstrap runs
      // for a handful of cycles as intended. Recover the state mean from the information vector first
      // (needs Y while it is still the information matrix), then invert the information matrix into a
      // covariance (Y -> P).
      if (static_cast<const MM&>(motionModel).getCov().determinant() > 100.0)
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

    std::cout << "  Measurement z:   (range=" << zRange << ", bearing=" << zBearing << ", doppler=" << zDoppler
              << ")  [ground truth: (" << gtRange << ", " << gtBearing << ", " << gtDoppler << ")]" << std::endl;
    printEstimate("  Corrected State:");
    std::cout << "  [ground truth position: (" << gtX << ", " << gtY << ")]" << std::endl;

    std::cout << filter_cov_str << std::endl;
    std::cout << static_cast<const MM&>(motionModel).getCov()() << std::endl;
    std::cout << std::endl;

    const auto [ex, ey, evx, evy, pXX, pXY, pYY] = getEstimate();
    csv << step << ',' << currentTime << ',' << gtX << ',' << gtY << ',' << ex << ',' << ey << ',' << evx << ',' << evy << ','
        << pXX << ',' << pXY << ',' << pYY << ',' << (useKalman ? 1 : 0) << ',' << egoWorldX << ',' << egoWorldY << ','
        << egoWorldPsi << ',' << targetWorldX << ',' << targetWorldY << ',' << zRange << ',' << zBearing << ',' << zDoppler
        << '\n';
  }

  std::cout << "Simulation completed successfully!" << std::endl;
  std::cout << "Final position: (" << motionModel.getX() << ", " << motionModel.getY() << ")" << std::endl;
  std::cout << "Final velocity: (" << motionModel.getVx() << ", " << motionModel.getVy() << ")" << std::endl;

  return 0;
}
