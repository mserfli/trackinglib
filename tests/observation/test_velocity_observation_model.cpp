#include <gtest/gtest.h>

#include "trackingLib/motion/motion_model_ca.h" // IWYU pragma: keep  (StateDefCA)
#include "trackingLib/motion/motion_model_cv.h" // IWYU pragma: keep  (StateDefCV)
#include "trackingLib/observation/velocity_observation_model.h"

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;
using StateDefCA     = tracking::motion::StateDefCA;

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

TEST(VelocityObservationModel, ctor_FromLists__Success) // NOLINT
{
  // clang-format off
  const auto obs = VelModel::FromLists({2.1, 0.9}, {
    {0.2, 0.0},
    {0.0, 0.3}
  });
  // clang-format on

  EXPECT_DOUBLE_EQ(obs[VelModel::MEAS_VX], 2.1);
  EXPECT_DOUBLE_EQ(obs[VelModel::MEAS_VY], 0.9);
  EXPECT_DOUBLE_EQ(obs(VelModel::MEAS_VX, VelModel::MEAS_VX), 0.2);
  EXPECT_DOUBLE_EQ(obs(VelModel::MEAS_VY, VelModel::MEAS_VY), 0.3);
}

TEST(VelocityObservationModel, predictMeasurement__ReturnsVelocity) // NOLINT
{
  const auto obs   = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  const auto state = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0}); // {X, VX, Y, VY}

  const auto predicted = obs.predictMeasurement(state);

  EXPECT_DOUBLE_EQ(predicted.at_unsafe(VelModel::MEAS_VX), 2.0);
  EXPECT_DOUBLE_EQ(predicted.at_unsafe(VelModel::MEAS_VY), 1.0);
}

TEST(VelocityObservationModel, computeJacobian__MatchesFiniteDifference) // NOLINT
{
  const auto obs   = VelModel::FromLists({0, 0}, {{1, 0}, {0, 1}});
  const auto state = VelModel::StateVec::FromList({10.0, 2.0, 5.0, 1.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-9);
}
