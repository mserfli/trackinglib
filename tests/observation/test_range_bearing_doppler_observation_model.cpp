#include <gtest/gtest.h>

#include "trackingLib/env/ego_motion.hpp"       // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.h" // IWYU pragma: keep  (StateDefCV)
#include "trackingLib/observation/range_bearing_doppler_observation_model.h"
#include "trackingLib/observation/sensor_mounting_pose.h"
#include <cmath>

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;

template <typename CovarianceMatrixPolicy_>
auto makeNoEgoMotion() -> tracking::env::EgoMotion<CovarianceMatrixPolicy_>
{
  using EgoMotionInst = tracking::env::EgoMotion<CovarianceMatrixPolicy_>;
  return EgoMotionInst(typename EgoMotionInst::InertialMotion{},
                       typename EgoMotionInst::Geometry{},
                       static_cast<typename CovarianceMatrixPolicy_::value_type>(1.0));
}

// instatiate all templates for full coverage report
template class tracking::observation::RangeBearingDopplerObservationModel<FullPolicy, StateDefCV>;
template class tracking::observation::RangeBearingDopplerObservationModel<FactoredPolicy, StateDefCV>;

using RbdModel = tracking::observation::RangeBearingDopplerObservationModel<FullPolicy, StateDefCV>;

namespace
{

/// \brief Position-only StateDef exercising the has_velocity_v == false branch
struct StateDefPosOnly
{
  enum
  {
    X = 0,
    Y,
    NUM_STATE_VARIABLES
  };
};

} // namespace

// instatiate the no-velocity branch for full coverage report
template class tracking::observation::RangeBearingDopplerObservationModel<FullPolicy, StateDefPosOnly>;

using RbdModelPosOnly = tracking::observation::RangeBearingDopplerObservationModel<FullPolicy, StateDefPosOnly>;

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

TEST(RangeBearingDopplerObservationModel, predictMeasurement__ReturnsPolarCoordinatesAndDoppler) // NOLINT
{
  // clang-format off
  const auto obs = RbdModel::FromLists({0, 0, 0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on
  const auto state     = RbdModel::StateVec::FromList({3.0, 1.0, 4.0, 2.0}); // {X, VX, Y, VY}
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  EXPECT_NEAR(predicted.at_unsafe(RbdModel::MEAS_RANGE), 5.0, 1e-12);
  EXPECT_NEAR(predicted.at_unsafe(RbdModel::MEAS_BEARING), std::atan2(4.0, 3.0), 1e-12);
  // doppler = (x*vx + y*vy) / range = (3*1 + 4*2) / 5 = 2.2
  EXPECT_NEAR(predicted.at_unsafe(RbdModel::MEAS_DOPPLER), 2.2, 1e-12);
}

TEST(RangeBearingDopplerObservationModel, computeJacobian__MatchesFiniteDifference) // NOLINT
{
  // clang-format off
  const auto obs = RbdModel::FromLists({0, 0, 0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on
  const auto state = RbdModel::StateVec::FromList({3.0, 1.0, 4.0, 2.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-7);
}

TEST(RangeBearingDopplerObservationModel, computeInnovation__WrapsBearing) // NOLINT
{
  const Testvalue_type pi = std::acos(-1.0);

  // clang-format off
  const auto obs = RbdModel::FromLists({5.0, pi - 0.1, 2.0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on
  const auto predicted = RbdModel::MeasurementVecFromList({5.0, -pi + 0.1, 1.5});

  const auto innovation = obs.computeInnovation(obs.getVec(), predicted);

  EXPECT_NEAR(innovation.at_unsafe(RbdModel::MEAS_RANGE), 0.0, 1e-12);
  EXPECT_NEAR(innovation.at_unsafe(RbdModel::MEAS_BEARING), -0.2, 1e-9);
  EXPECT_NEAR(innovation.at_unsafe(RbdModel::MEAS_DOPPLER), 0.5, 1e-12);
}

TEST(RangeBearingDopplerObservationModel, predictMeasurement__DopplerZeroWithoutVelocity) // NOLINT
{
  // clang-format off
  const auto obs = RbdModelPosOnly::FromLists({0, 0, 0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on
  const auto state     = RbdModelPosOnly::StateVec::FromList({3.0, 4.0}); // {X, Y}
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  EXPECT_NEAR(predicted.at_unsafe(RbdModelPosOnly::MEAS_RANGE), 5.0, 1e-12);
  EXPECT_NEAR(predicted.at_unsafe(RbdModelPosOnly::MEAS_BEARING), std::atan2(4.0, 3.0), 1e-12);
  // without velocity states the doppler cannot be predicted: it is zero by definition
  EXPECT_NEAR(predicted.at_unsafe(RbdModelPosOnly::MEAS_DOPPLER), 0.0, 1e-12);
}

TEST(RangeBearingDopplerObservationModel, computeJacobian__DopplerRowZeroWithoutVelocity) // NOLINT
{
  // clang-format off
  const auto obs = RbdModelPosOnly::FromLists({0, 0, 0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on
  const auto state     = RbdModelPosOnly::StateVec::FromList({3.0, 4.0});
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  typename RbdModelPosOnly::JacobianMatrix H{};
  obs.computeJacobian(H, state, egoMotion);

  // range/bearing rows are still populated
  EXPECT_NEAR(H.at_unsafe(RbdModelPosOnly::MEAS_RANGE, StateDefPosOnly::X), 3.0 / 5.0, 1e-12);
  EXPECT_NEAR(H.at_unsafe(RbdModelPosOnly::MEAS_BEARING, StateDefPosOnly::Y), 3.0 / 25.0, 1e-12);
  // the doppler row is all-zero: zero gain column, the doppler contributes nothing to the update
  for (auto col = 0; col < RbdModelPosOnly::DimX; ++col)
  {
    EXPECT_NEAR(H.at_unsafe(RbdModelPosOnly::MEAS_DOPPLER, col), 0.0, 1e-12) << "col " << col;
  }
}

TEST(RangeBearingDopplerObservationModel, predictMeasurement__AppliesSensorMountingPose) // NOLINT
{
  // clang-format off
  const auto pose = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.0, 0.0, std::acos(-1.0) / 2.0);
  const auto obs  = RbdModel::FromLists({0, 0, 0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  }, pose);
  // clang-format on
  const auto state     = RbdModel::StateVec::FromList({3.0, 1.0, 4.0, 2.0}); // {X, VX, Y, VY}
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  // p - t = (2, 4), rotated into the sensor frame by -90deg: (4, -2); v = (1, 2) rotated: (2, -1)
  EXPECT_NEAR(predicted.at_unsafe(RbdModel::MEAS_RANGE), std::sqrt(20.0), 1e-9);
  EXPECT_NEAR(predicted.at_unsafe(RbdModel::MEAS_BEARING), std::atan2(-2.0, 4.0), 1e-9);
  // doppler = (ps.x*vs.x + ps.y*vs.y) / range = (4*2 + -2*-1) / sqrt(20) = 10 / sqrt(20)
  EXPECT_NEAR(predicted.at_unsafe(RbdModel::MEAS_DOPPLER), 10.0 / std::sqrt(20.0), 1e-9);
}

TEST(RangeBearingDopplerObservationModel, computeJacobian__MatchesFiniteDifferenceWithSensorMountingPose) // NOLINT
{
  // clang-format off
  const auto pose = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.0, 0.0, std::acos(-1.0) / 2.0);
  const auto obs  = RbdModel::FromLists({0, 0, 0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  }, pose);
  // clang-format on
  const auto state = RbdModel::StateVec::FromList({3.0, 1.0, 4.0, 2.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-7);
}

TEST(RangeBearingDopplerObservationModel, predictMeasurement__CompensatesEgoMotion) // NOLINT
{
  // clang-format off
  const auto obs = RbdModel::FromLists({0, 0, 0}, {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on
  const auto state = RbdModel::StateVec::FromList({3.0, 1.0, 4.0, 2.0}); // {X, VX, Y, VY}

  // nonzero ego COG speed and yaw rate: the sensor (mounted at the tracking-frame origin) itself
  // moves at (v, 0) + w x r, r = (0 - distCog2Ego, 0) = (v, -w*distCog2Ego)
  using EgoMotionInst = RbdModel::EgoMotionType;
  EgoMotionInst::InertialMotion motion{};
  motion.v = 5.0;
  motion.w = 0.2;
  EgoMotionInst::Geometry geometry{};
  geometry.distCog2Ego = 10.0;
  const auto egoMotion = EgoMotionInst(motion, geometry, static_cast<Testvalue_type>(1.0));

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  // egoVel = (5, -0.2*10) = (5, -2); vRel = (1 - 5, 2 - (-2)) = (-4, 4)
  // doppler = (x*vxRel + y*vyRel) / range = (3*-4 + 4*4) / 5 = 0.8
  EXPECT_NEAR(predicted.at_unsafe(RbdModel::MEAS_DOPPLER), 0.8, 1e-9);

  // the naive (uncompensated) formula would have given (3*1 + 4*2) / 5 = 2.2 - confirm the fix
  // actually changes the result rather than accidentally cancelling out
  EXPECT_FALSE(std::abs(predicted.at_unsafe(RbdModel::MEAS_DOPPLER) - 2.2) < 1e-9);
}
