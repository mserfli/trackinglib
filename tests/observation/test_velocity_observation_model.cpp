#include <gtest/gtest.h>

#include "trackingLib/env/ego_motion.hpp"       // IWYU pragma: keep
#include "trackingLib/motion/motion_model_ca.h" // IWYU pragma: keep  (StateDefCA)
#include "trackingLib/motion/motion_model_cv.h" // IWYU pragma: keep  (StateDefCV)
#include "trackingLib/observation/range_bearing_doppler_observation_model.h"
#include "trackingLib/observation/sensor_mounting_pose.h"
#include "trackingLib/observation/velocity_observation_model.h"
#include <cmath>

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;
using StateDefCA     = tracking::motion::StateDefCA;

template <typename CovarianceMatrixPolicy_>
auto makeNoEgoMotion() -> tracking::env::EgoMotion<CovarianceMatrixPolicy_>
{
  using EgoMotionInst = tracking::env::EgoMotion<CovarianceMatrixPolicy_>;
  return EgoMotionInst(typename EgoMotionInst::InertialMotion{},
                       typename EgoMotionInst::Geometry{},
                       static_cast<typename CovarianceMatrixPolicy_::value_type>(1.0));
}

// A genuinely moving + turning platform (v != 0, w != 0) with a lever arm (distCog2Ego != 0), so
// EgoMotion::getVelocityAt() at a mounted sensor returns a nonzero platform-relative velocity.
template <typename CovarianceMatrixPolicy_>
auto makeMovingTurningEgoMotion() -> tracking::env::EgoMotion<CovarianceMatrixPolicy_>
{
  using EgoMotionInst  = tracking::env::EgoMotion<CovarianceMatrixPolicy_>;
  using vt             = typename CovarianceMatrixPolicy_::value_type;
  auto motion          = typename EgoMotionInst::InertialMotion{};
  motion.v             = static_cast<vt>(3.0);
  motion.a             = static_cast<vt>(0.5);
  motion.w             = static_cast<vt>(0.2);
  auto geometry        = typename EgoMotionInst::Geometry{};
  geometry.distCog2Ego = static_cast<vt>(1.0);
  return EgoMotionInst(motion, geometry, static_cast<vt>(1.0));
}

// instatiate all templates for full coverage report
template class tracking::observation::VelocityObservationModel<FullPolicy, StateDefCV>;
template class tracking::observation::VelocityObservationModel<FactoredPolicy, StateDefCV>;
template class tracking::observation::VelocityObservationModel<FullPolicy, StateDefCA>;

using VelModel = tracking::observation::VelocityObservationModel<FullPolicy, StateDefCV>;

namespace
{

/// \brief Compare the analytic Jacobian against a central finite difference of predictMeasurement
template <typename ObservationModel_>
void expectJacobianMatchesFiniteDifference(const ObservationModel_&                    obs,
                                           const typename ObservationModel_::StateVec& state,
                                           const Testvalue_type                        tol)
{
  using EgoMotionInst  = typename ObservationModel_::EgoMotionType;
  const auto egoMotion = EgoMotionInst(typename EgoMotionInst::InertialMotion{},
                                       typename EgoMotionInst::Geometry{},
                                       static_cast<typename EgoMotionInst::value_type>(1.0));

  typename ObservationModel_::JacobianMatrix H{};
  obs.computeJacobian(H, state, egoMotion);

  const Testvalue_type eps = 1e-6;
  for (auto col = 0; col < ObservationModel_::DimX; ++col)
  {
    auto statePlus  = state;
    auto stateMinus = state;
    statePlus.at_unsafe(col) += eps;
    stateMinus.at_unsafe(col) -= eps;
    const auto hPlus  = obs.predictMeasurement(statePlus, egoMotion);
    const auto hMinus = obs.predictMeasurement(stateMinus, egoMotion);
    for (auto row = 0; row < ObservationModel_::DimZ; ++row)
    {
      const Testvalue_type fd = (hPlus.at_unsafe(row) - hMinus.at_unsafe(row)) / (2 * eps);
      EXPECT_NEAR(H.at_unsafe(row, col), fd, tol) << "row " << row << " col " << col;
    }
  }
}

} // namespace

TEST(VelocityObservationModel, ctor_FromLists__Success) // NOLINT
{
  // clang-format off
  const auto obs = VelModel::FromLists({2.1, 0.9}, {
    {0.2, 0.0},
    {0.0, 0.3}
  }).value();
  // clang-format on

  EXPECT_DOUBLE_EQ(obs[VelModel::MEAS_VX], 2.1);
  EXPECT_DOUBLE_EQ(obs[VelModel::MEAS_VY], 0.9);
  EXPECT_DOUBLE_EQ(obs(VelModel::MEAS_VX, VelModel::MEAS_VX), 0.2);
  EXPECT_DOUBLE_EQ(obs(VelModel::MEAS_VY, VelModel::MEAS_VY), 0.3);
}

TEST(VelocityObservationModel, predictMeasurement__ReturnsVelocity) // NOLINT
{
  const auto obs       = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}}).value();
  const auto state     = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0}); // {X, VX, Y, VY}
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  EXPECT_DOUBLE_EQ(predicted.at_unsafe(VelModel::MEAS_VX), 2.0);
  EXPECT_DOUBLE_EQ(predicted.at_unsafe(VelModel::MEAS_VY), 1.0);
}

TEST(VelocityObservationModel, computeJacobian__MatchesFiniteDifference) // NOLINT
{
  const auto obs   = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}}).value();
  const auto state = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-9);
}

TEST(VelocityObservationModel, predictMeasurement__AppliesSensorMountingPose) // NOLINT
{
  // static mount: translation has no lever-arm effect, only the yaw rotates velocity
  const auto pose      = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.0, 0.0, std::acos(-1.0) / 2.0);
  const auto obs       = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}}, pose).value();
  const auto state     = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0}); // {X, VX, Y, VY}
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  // v rotated by -90deg: (2, 1) -> (1, -2)
  EXPECT_NEAR(predicted.at_unsafe(VelModel::MEAS_VX), 1.0, 1e-9);
  EXPECT_NEAR(predicted.at_unsafe(VelModel::MEAS_VY), -2.0, 1e-9);
}

TEST(VelocityObservationModel, computeJacobian__MatchesFiniteDifferenceWithSensorMountingPose) // NOLINT
{
  const auto pose  = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.0, 0.0, std::acos(-1.0) / 2.0);
  const auto obs   = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}}, pose).value();
  const auto state = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-7);
}

// ---------------------------------------------------------------------------------------------
// The velocity model subtracts the sensor's ego (lever-arm)
// velocity, mirroring the doppler model - a mounted velocity sensor on a moving/turning platform
// reports the target velocity relative to the sensor. The regression tests below reuse the exact
// primitives the doppler model uses (EgoMotion::getVelocityAt() +
// SensorMountingPose::directionToSensorFrame()) as the oracle, matching
// RangeBearingDopplerObservationModel::predictMeasurementSensorFrame().

using DopplerModel = tracking::observation::RangeBearingDopplerObservationModel<FullPolicy, StateDefCV>;

// regression: the velocity model subtracts the ego lever-arm velocity like the doppler model does.
TEST(VelocityObservationModel, predictMeasurement_MovingTurningPlatform__SubtractsEgoLeverArmVelocity) // NOLINT
{
  const auto pose      = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.5, 0.5, std::acos(-1.0) / 6.0);
  const auto obs       = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}}, pose).value();
  const auto state     = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0});
  const auto egoMotion = makeMovingTurningEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  // oracle: mirror RangeBearingDopplerObservationModel::predictMeasurementSensorFrame's ego term
  const auto egoVelMount    = egoMotion.getVelocityAt(pose.tx(), pose.ty());
  const auto egoVelSensor   = pose.directionToSensorFrame(egoVelMount.x(), egoVelMount.y());
  const auto stateVelSensor = pose.directionToSensorFrame(state.at_unsafe(StateDefCV::VX), state.at_unsafe(StateDefCV::VY));

  EXPECT_NEAR(predicted.at_unsafe(VelModel::MEAS_VX), stateVelSensor.x() - egoVelSensor.x(), 1e-9);
  EXPECT_NEAR(predicted.at_unsafe(VelModel::MEAS_VY), stateVelSensor.y() - egoVelSensor.y(), 1e-9);
}

// consistency: both doppler and velocity now respond to the platform's ego motion (each subtracts
// the lever-arm velocity), so neither is invariant to a moving vs. static platform.
TEST(VelocityObservationModel, predictMeasurement_DopplerVsVelocity_EgoHandling__BothRespond) // NOLINT
{
  const auto pose       = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.5, 0.5, std::acos(-1.0) / 6.0);
  const auto velObs     = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}}, pose).value();
  const auto dopplerObs = DopplerModel::FromLists({0, 0, 0}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, pose).value();
  const auto state      = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0});
  const auto movingEgo  = makeMovingTurningEgoMotion<FullPolicy>();
  const auto noEgo      = makeNoEgoMotion<FullPolicy>();

  // doppler diverges between a moving and a static platform (it subtracts the ego velocity)
  const auto dopplerMoving = dopplerObs.predictMeasurement(state, movingEgo).at_unsafe(DopplerModel::MEAS_DOPPLER);
  const auto dopplerStatic = dopplerObs.predictMeasurement(state, noEgo).at_unsafe(DopplerModel::MEAS_DOPPLER);
  EXPECT_GT(std::abs(dopplerMoving - dopplerStatic), 1e-6);

  // the velocity model now also diverges between a moving and a static platform (it too subtracts
  // the ego velocity), consistent with the doppler model
  const auto velMoving = velObs.predictMeasurement(state, movingEgo);
  const auto velStatic = velObs.predictMeasurement(state, noEgo);
  EXPECT_GT(std::abs(velMoving.at_unsafe(VelModel::MEAS_VX) - velStatic.at_unsafe(VelModel::MEAS_VX)), 1e-6);
  EXPECT_GT(std::abs(velMoving.at_unsafe(VelModel::MEAS_VY) - velStatic.at_unsafe(VelModel::MEAS_VY)), 1e-6);
}
