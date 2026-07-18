#include <gtest/gtest.h>

#include "trackingLib/motion/motion_model_ca.h" // IWYU pragma: keep  (StateDefCA)
#include "trackingLib/motion/motion_model_cv.h" // IWYU pragma: keep  (StateDefCV)
#include "trackingLib/motion/state_def_traits.h"
#include "trackingLib/observation/position_observation_model.h"

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;
using StateDefCA     = tracking::motion::StateDefCA;

// instatiate all templates for full coverage report
template class tracking::observation::PositionObservationModel<FullPolicy, StateDefCV>;
template class tracking::observation::PositionObservationModel<FactoredPolicy, StateDefCV>;
template class tracking::observation::PositionObservationModel<FullPolicy, StateDefCA>;

using PosModel = tracking::observation::PositionObservationModel<FullPolicy, StateDefCV>;

// compile-time capability checks of the state definition traits
static_assert(tracking::motion::has_position_v<StateDefCV>, "StateDefCV must provide a position");
static_assert(tracking::motion::has_velocity_v<StateDefCV>, "StateDefCV must provide a velocity");
static_assert(!tracking::motion::has_acceleration_v<StateDefCV>, "StateDefCV must not provide an acceleration");
static_assert(tracking::motion::has_acceleration_v<StateDefCA>, "StateDefCA must provide an acceleration");
static_assert(tracking::motion::state_dimension_v<StateDefCV> == 4, "StateDefCV must have 4 state variables");
static_assert(tracking::motion::state_dimension_v<StateDefCA> == 6, "StateDefCA must have 6 state variables");

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

TEST(PositionObservationModel, ctor_FromLists__Success) // NOLINT
{
  // clang-format off
  const auto obs = PosModel::FromLists({10.5, 5.2}, {
    {0.5, 0.0},
    {0.0, 0.4}
  });
  // clang-format on

  EXPECT_DOUBLE_EQ(obs[PosModel::MEAS_X], 10.5);
  EXPECT_DOUBLE_EQ(obs[PosModel::MEAS_Y], 5.2);
  EXPECT_DOUBLE_EQ(obs(PosModel::MEAS_X, PosModel::MEAS_X), 0.5);
  EXPECT_DOUBLE_EQ(obs(PosModel::MEAS_Y, PosModel::MEAS_Y), 0.4);
}

TEST(PositionObservationModel, getDim__ReturnsDimZ) // NOLINT
{
  const auto obs = PosModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  EXPECT_EQ(obs.getDim(), 2);
  EXPECT_EQ(PosModel::DimZ, 2);
  EXPECT_EQ(PosModel::DimX, 4);
}

TEST(PositionObservationModel, predictMeasurement__ReturnsPosition) // NOLINT
{
  const auto obs   = PosModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  const auto state = PosModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0}); // {X, VX, Y, VY}

  const auto predicted = obs.predictMeasurement(state);

  EXPECT_DOUBLE_EQ(predicted.at_unsafe(PosModel::MEAS_X), 10.0);
  EXPECT_DOUBLE_EQ(predicted.at_unsafe(PosModel::MEAS_Y), 5.0);
}

TEST(PositionObservationModel, computeJacobian__MatchesFiniteDifference) // NOLINT
{
  const auto obs   = PosModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  const auto state = PosModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-9);
}

TEST(PositionObservationModel, computeInnovation__ComponentWiseDifference) // NOLINT
{
  const auto obs       = PosModel::FromLists({10.5, 5.2}, {{1, 0}, {0, 1}});
  const auto predicted = PosModel::MeasurementVecFromList({10.0, 5.0});

  const auto innovation = obs.computeInnovation(obs.getVec(), predicted);

  EXPECT_NEAR(innovation.at_unsafe(PosModel::MEAS_X), 0.5, 1e-12);
  EXPECT_NEAR(innovation.at_unsafe(PosModel::MEAS_Y), 0.2, 1e-12);
}
