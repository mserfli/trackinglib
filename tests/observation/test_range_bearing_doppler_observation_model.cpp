#include <gtest/gtest.h>

#include "trackingLib/motion/motion_model_cv.h" // IWYU pragma: keep  (StateDefCV)
#include "trackingLib/observation/range_bearing_doppler_observation_model.h"
#include <cmath>

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;

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
  typename ObservationModel_::JacobianMatrix H{};
  obs.computeJacobian(H, state);

  const Testvalue_type eps = 1e-6;
  for (auto col = 0; col < ObservationModel_::DimX; ++col)
  {
    auto statePlus  = state;
    auto stateMinus = state;
    statePlus.at_unsafe(col) += eps;
    stateMinus.at_unsafe(col) -= eps;
    const auto hPlus  = obs.predictMeasurement(statePlus);
    const auto hMinus = obs.predictMeasurement(stateMinus);
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
  const auto state = RbdModel::StateVec::FromList({3.0, 1.0, 4.0, 2.0}); // {X, VX, Y, VY}

  const auto predicted = obs.predictMeasurement(state);

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
  const auto state = RbdModelPosOnly::StateVec::FromList({3.0, 4.0}); // {X, Y}

  const auto predicted = obs.predictMeasurement(state);

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
  const auto state = RbdModelPosOnly::StateVec::FromList({3.0, 4.0});

  typename RbdModelPosOnly::JacobianMatrix H{};
  obs.computeJacobian(H, state);

  // range/bearing rows are still populated
  EXPECT_NEAR(H.at_unsafe(RbdModelPosOnly::MEAS_RANGE, StateDefPosOnly::X), 3.0 / 5.0, 1e-12);
  EXPECT_NEAR(H.at_unsafe(RbdModelPosOnly::MEAS_BEARING, StateDefPosOnly::Y), 3.0 / 25.0, 1e-12);
  // the doppler row is all-zero: zero gain column, the doppler contributes nothing to the update
  for (auto col = 0; col < RbdModelPosOnly::DimX; ++col)
  {
    EXPECT_NEAR(H.at_unsafe(RbdModelPosOnly::MEAS_DOPPLER, col), 0.0, 1e-12) << "col " << col;
  }
}
