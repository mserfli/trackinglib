#include <gtest/gtest.h>

#include "trackingLib/env/ego_motion.hpp"       // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.h" // IWYU pragma: keep  (StateDefCV)
#include "trackingLib/observation/range_bearing_observation_model.h"
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
template class tracking::observation::RangeBearingObservationModel<FullPolicy, StateDefCV>;
template class tracking::observation::RangeBearingObservationModel<FactoredPolicy, StateDefCV>;

using RbModel = tracking::observation::RangeBearingObservationModel<FullPolicy, StateDefCV>;

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

TEST(RangeBearingObservationModel, predictMeasurement__ReturnsPolarCoordinates) // NOLINT
{
  const auto obs       = RbModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  const auto state     = RbModel::StateVec::FromList({3.0, 2.0, 4.0, 1.0}); // {X, VX, Y, VY}
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  EXPECT_NEAR(predicted.at_unsafe(RbModel::MEAS_RANGE), 5.0, 1e-12);
  EXPECT_NEAR(predicted.at_unsafe(RbModel::MEAS_BEARING), std::atan2(4.0, 3.0), 1e-12);
}

TEST(RangeBearingObservationModel, computeJacobian__MatchesFiniteDifference) // NOLINT
{
  const auto obs   = RbModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  const auto state = RbModel::StateVec::FromList({3.0, 2.0, 4.0, 1.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-7);
}

TEST(RangeBearingObservationModel, computeJacobian__NoDivisionByZeroAtOrigin) // NOLINT
{
  const auto obs       = RbModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  const auto state     = RbModel::StateVec::FromList({0.0, 0.0, 0.0, 0.0});
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  typename RbModel::JacobianMatrix H{};
  obs.computeJacobian(H, state, egoMotion);

  for (auto row = 0; row < RbModel::DimZ; ++row)
  {
    for (auto col = 0; col < RbModel::DimX; ++col)
    {
      EXPECT_TRUE(std::isfinite(H.at_unsafe(row, col))) << "row " << row << " col " << col;
    }
  }
}

TEST(RangeBearingObservationModel, computeInnovation__WrapsBearing) // NOLINT
{
  const Testvalue_type pi = std::acos(-1.0);

  // measured bearing just below +pi, predicted just above -pi: the raw difference is ~2*pi
  // but the true angular error is only -0.2 rad
  const auto obs       = RbModel::FromLists({5.0, pi - 0.1}, {{1, 0}, {0, 1}});
  const auto predicted = RbModel::MeasurementVecFromList({5.0, -pi + 0.1});

  const auto innovation = obs.computeInnovation(obs.getVec(), predicted);

  EXPECT_NEAR(innovation.at_unsafe(RbModel::MEAS_RANGE), 0.0, 1e-12);
  EXPECT_NEAR(innovation.at_unsafe(RbModel::MEAS_BEARING), -0.2, 1e-9);
}

TEST(RangeBearingObservationModel, predictMeasurement__AppliesSensorMountingPose) // NOLINT
{
  const auto pose      = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.0, 0.0, std::acos(-1.0) / 2.0);
  const auto obs       = RbModel::FromLists({0, 0}, {{1, 0}, {0, 1}}, pose);
  const auto state     = RbModel::StateVec::FromList({3.0, 2.0, 4.0, 1.0}); // {X, VX, Y, VY}
  const auto egoMotion = makeNoEgoMotion<FullPolicy>();

  const auto predicted = obs.predictMeasurement(state, egoMotion);

  // p - t = (2, 4), rotated into the sensor frame by -90deg: (4, -2)
  EXPECT_NEAR(predicted.at_unsafe(RbModel::MEAS_RANGE), std::sqrt(20.0), 1e-9);
  EXPECT_NEAR(predicted.at_unsafe(RbModel::MEAS_BEARING), std::atan2(-2.0, 4.0), 1e-9);
}

TEST(RangeBearingObservationModel, computeJacobian__MatchesFiniteDifferenceWithSensorMountingPose) // NOLINT
{
  const auto pose  = tracking::observation::SensorMountingPose<Testvalue_type>::FromValues(1.0, 0.0, std::acos(-1.0) / 2.0);
  const auto obs   = RbModel::FromLists({0, 0}, {{1, 0}, {0, 1}}, pose);
  const auto state = RbModel::StateVec::FromList({3.0, 2.0, 4.0, 1.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-7);
}
