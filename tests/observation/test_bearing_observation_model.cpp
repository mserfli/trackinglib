#include <gtest/gtest.h>

#include "trackingLib/motion/motion_model_cv.h" // IWYU pragma: keep  (StateDefCV)
#include "trackingLib/observation/bearing_observation_model.h"
#include <cmath>

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;

// instatiate all templates for full coverage report
template class tracking::observation::BearingObservationModel<FullPolicy, StateDefCV>;
template class tracking::observation::BearingObservationModel<FactoredPolicy, StateDefCV>;

using BearingModel = tracking::observation::BearingObservationModel<FullPolicy, StateDefCV>;

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

TEST(BearingObservationModel, predictMeasurement__ReturnsBearing) // NOLINT
{
  const auto obs   = BearingModel::FromLists({0}, {{1}});
  const auto state = BearingModel::StateVec::FromList({3.0, 2.0, 4.0, 1.0}); // {X, VX, Y, VY}

  const auto predicted = obs.predictMeasurement(state);

  EXPECT_NEAR(predicted.at_unsafe(BearingModel::MEAS_BEARING), std::atan2(4.0, 3.0), 1e-12);
}

TEST(BearingObservationModel, computeJacobian__MatchesFiniteDifference) // NOLINT
{
  const auto obs   = BearingModel::FromLists({0}, {{1}});
  const auto state = BearingModel::StateVec::FromList({3.0, 2.0, 4.0, 1.0});

  expectJacobianMatchesFiniteDifference(obs, state, 1e-7);
}

TEST(BearingObservationModel, computeJacobian__NoDivisionByZeroAtOrigin) // NOLINT
{
  const auto obs   = BearingModel::FromLists({0}, {{1}});
  const auto state = BearingModel::StateVec::FromList({0.0, 0.0, 0.0, 0.0});

  typename BearingModel::JacobianMatrix H{};
  obs.computeJacobian(H, state);

  for (auto row = 0; row < BearingModel::DimZ; ++row)
  {
    for (auto col = 0; col < BearingModel::DimX; ++col)
    {
      EXPECT_TRUE(std::isfinite(H.at_unsafe(row, col))) << "row " << row << " col " << col;
    }
  }
}

TEST(BearingObservationModel, computeInnovation__WrapsBearingAcrossPi) // NOLINT
{
  const Testvalue_type pi = std::acos(-1.0);

  // measured bearing just below +pi, predicted just above -pi: the raw difference is ~2*pi
  // but the true angular error is only -0.2 rad
  const auto obs       = BearingModel::FromLists({pi - 0.1}, {{1}});
  const auto predicted = BearingModel::MeasurementVecFromList({-pi + 0.1});

  const auto innovation = obs.computeInnovation(obs.getVec(), predicted);

  EXPECT_NEAR(innovation.at_unsafe(BearingModel::MEAS_BEARING), -0.2, 1e-9);
}
