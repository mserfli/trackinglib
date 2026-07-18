#include <gtest/gtest.h>

#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include "trackingLib/observation/position_observation_model.h"
#include "trackingLib/observation/velocity_observation_model.h"
#include <cmath>

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;

using MMFull  = tracking::motion::MotionModelCV<FullPolicy>;
using MMFact  = tracking::motion::MotionModelCV<FactoredPolicy>;
using PosFull = tracking::observation::PositionObservationModel<FullPolicy, StateDefCV>;
using PosFact = tracking::observation::PositionObservationModel<FactoredPolicy, StateDefCV>;
using VelFull = tracking::observation::VelocityObservationModel<FullPolicy, StateDefCV>;
using VelFact = tracking::observation::VelocityObservationModel<FactoredPolicy, StateDefCV>;

using KalmanFull      = tracking::filter::KalmanFilter<FullPolicy>;
using KalmanFact      = tracking::filter::KalmanFilter<FactoredPolicy>;
using InformationFull = tracking::filter::InformationFilter<FullPolicy>;
using InformationFact = tracking::filter::InformationFilter<FactoredPolicy>;

namespace update_mode = tracking::filter::update_mode;

namespace
{

/// \brief Compare state vector and (composed) state covariance of two motion models
template <typename ModelA_, typename ModelB_>
void expectSamePosterior(const ModelA_& actual, const ModelB_& expected, const Testvalue_type tolVec, const Testvalue_type tolCov)
{
  for (auto row = 0; row < StateDefCV::NUM_STATE_VARIABLES; ++row)
  {
    EXPECT_NEAR(actual._vec.at_unsafe(row), expected._vec.at_unsafe(row), tolVec) << "vec row " << row;
    for (auto col = 0; col < StateDefCV::NUM_STATE_VARIABLES; ++col)
    {
      EXPECT_NEAR(actual._cov.at_unsafe(row, col), expected._cov.at_unsafe(row, col), tolCov)
          << "cov row " << row << " col " << col;
    }
  }
}

/// \brief Prior used across the cross-check tests (correlated, positive definite)
template <typename MM_>
auto makePrior() -> MM_
{
  // clang-format off
  return MM_::FromLists({10, 2, 5, 1}, {
    {4.0, 0.5, 0.2, 0.0},
    {0.5, 1.0, 0.0, 0.1},
    {0.2, 0.0, 2.0, 0.3},
    {0.0, 0.1, 0.3, 0.5}
  });
  // clang-format on
}

} // namespace

TEST(MeasurementUpdate, update_KalmanFullBlock__MatchesHandComputedPosterior) // NOLINT
{
  // diagonal prior so the posterior is hand-computable per axis:
  // S = diag(4+1, 2+0.5) ; K = diag-selective with K_x = 4/5, K_y = 2/2.5
  // clang-format off
  auto mm = MMFull::FromLists({10, 2, 5, 1}, {
    {4.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 2.0, 0.0},
    {0.0, 0.0, 0.0, 0.5}
  });
  const auto obs = PosFull::FromLists({10.6, 5.3}, {
    {1.0, 0.0},
    {0.0, 0.5}
  });
  // clang-format on
  const KalmanFull filter{};

  mm.update(filter, obs);

  const Testvalue_type tol = 1e-12;
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::X), 10.0 + 0.8 * 0.6, tol);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::VX), 2.0, tol);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::Y), 5.0 + 0.8 * 0.3, tol);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::VY), 1.0, tol);

  EXPECT_NEAR(mm._cov.at_unsafe(StateDefCV::X, StateDefCV::X), (1.0 - 0.8) * 4.0, tol);
  EXPECT_NEAR(mm._cov.at_unsafe(StateDefCV::VX, StateDefCV::VX), 1.0, tol);
  EXPECT_NEAR(mm._cov.at_unsafe(StateDefCV::Y, StateDefCV::Y), (1.0 - 0.8) * 2.0, tol);
  EXPECT_NEAR(mm._cov.at_unsafe(StateDefCV::VY, StateDefCV::VY), 0.5, tol);
  EXPECT_NEAR(mm._cov.at_unsafe(StateDefCV::X, StateDefCV::Y), 0.0, tol);
}

TEST(MeasurementUpdate, update_KalmanFullSequentialVsBlock__SamePosterior) // NOLINT
{
  auto mmBlock      = makePrior<MMFull>();
  auto mmSequential = makePrior<MMFull>();
  // clang-format off
  const auto obs = PosFull::FromLists({10.6, 5.3}, {
    {1.0, 0.0},
    {0.0, 0.5}
  });
  // clang-format on
  const KalmanFull filter{};

  mmBlock.update<update_mode::Block>(filter, obs);
  mmSequential.update<update_mode::Sequential>(filter, obs);

  expectSamePosterior(mmSequential, mmBlock, 1e-10, 1e-10);
}

TEST(MeasurementUpdate, update_KalmanFactoredVsFull__SamePosterior) // NOLINT
{
  auto mmFull = makePrior<MMFull>();
  auto mmFact = makePrior<MMFact>();
  // clang-format off
  const auto obsFull = PosFull::FromLists({10.6, 5.3}, {
    {1.0, 0.0},
    {0.0, 0.5}
  });
  const auto obsFact = PosFact::FromLists({10.6, 5.3}, {
    {1.0, 0.0},
    {0.0, 0.5}
  });
  // clang-format on

  mmFull.update(KalmanFull{}, obsFull);
  mmFact.update(KalmanFact{}, obsFact);

  expectSamePosterior(mmFact, mmFull, 1e-9, 1e-9);
}

TEST(MeasurementUpdate, update_InformationVsKalman__SamePosterior) // NOLINT
{
  auto mmKalman = makePrior<MMFull>();
  auto mmInfo   = makePrior<MMFull>();
  // clang-format off
  const auto obs = PosFull::FromLists({10.6, 5.3}, {
    {1.0, 0.0},
    {0.0, 0.5}
  });
  // clang-format on

  // transform the information model into information space: Y = inv(P), y = Y*x
  mmInfo._cov = mmInfo._cov.inverse().value();
  mmInfo._vec = static_cast<typename MMFull::StateVec>(mmInfo._cov() * mmInfo._vec);

  mmKalman.update(KalmanFull{}, obs);
  mmInfo.update(InformationFull{}, obs);

  // transform the information model back into state space for comparison
  mmInfo.convertStateVecIntoStateSpace();
  mmInfo._cov = mmInfo._cov.inverse().value();

  expectSamePosterior(mmInfo, mmKalman, 1e-8, 1e-8);
}

TEST(MeasurementUpdate, update_InformationFactoredVsKalmanFactored__SamePosterior) // NOLINT
{
  auto mmKalman = makePrior<MMFact>();
  auto mmInfo   = makePrior<MMFact>();
  // clang-format off
  const auto obs = PosFact::FromLists({10.6, 5.3}, {
    {1.0, 0.0},
    {0.0, 0.5}
  });
  // clang-format on

  // transform the information model into information space: Y = inv(P), y = Y*x
  mmInfo._cov = mmInfo._cov.inverse().value();
  mmInfo._vec = static_cast<typename MMFact::StateVec>(mmInfo._cov() * mmInfo._vec);

  mmKalman.update(KalmanFact{}, obs);
  mmInfo.update(InformationFact{}, obs);

  // transform the information model back into state space for comparison
  mmInfo.convertStateVecIntoStateSpace();
  mmInfo._cov = mmInfo._cov.inverse().value();

  expectSamePosterior(mmInfo, mmKalman, 1e-8, 1e-8);
}

TEST(MeasurementUpdate, update_KalmanFullSequentialCorrelatedR_VsBlock__SamePosterior) // NOLINT
{
  auto mmBlock      = makePrior<MMFull>();
  auto mmSequential = makePrior<MMFull>();
  // correlated measurement covariance: the sequential path must decorrelate via R = U*D*U'
  // clang-format off
  const auto obs = PosFull::FromLists({10.6, 5.3}, {
    {1.0, 0.3},
    {0.3, 0.5}
  });
  // clang-format on
  const KalmanFull filter{};

  mmBlock.update<update_mode::Block>(filter, obs);
  mmSequential.update<update_mode::Sequential>(filter, obs);

  expectSamePosterior(mmSequential, mmBlock, 1e-10, 1e-10);
}

TEST(MeasurementUpdate, update_KalmanFactoredCorrelatedR_VsFullBlock__SamePosterior) // NOLINT
{
  auto mmFull = makePrior<MMFull>();
  auto mmFact = makePrior<MMFact>();
  // correlated measurement covariance: the factored sequential path must decorrelate via U and D
  // clang-format off
  const auto obsFull = PosFull::FromLists({10.6, 5.3}, {
    {1.0, 0.3},
    {0.3, 0.5}
  });
  const auto obsFact = PosFact::FromLists({10.6, 5.3}, {
    {1.0, 0.3},
    {0.3, 0.5}
  });
  // clang-format on

  mmFull.update(KalmanFull{}, obsFull);
  mmFact.update(KalmanFact{}, obsFact);

  expectSamePosterior(mmFact, mmFull, 1e-9, 1e-9);
}

TEST(MeasurementUpdate, update_InformationFactoredCorrelatedR_VsKalman__SamePosterior) // NOLINT
{
  auto mmKalman = makePrior<MMFact>();
  auto mmInfo   = makePrior<MMFact>();
  // clang-format off
  const auto obs = PosFact::FromLists({10.6, 5.3}, {
    {1.0, 0.3},
    {0.3, 0.5}
  });
  // clang-format on

  // transform the information model into information space: Y = inv(P), y = Y*x
  mmInfo._cov = mmInfo._cov.inverse().value();
  mmInfo._vec = static_cast<typename MMFact::StateVec>(mmInfo._cov() * mmInfo._vec);

  mmKalman.update(KalmanFact{}, obs);
  mmInfo.update(InformationFact{}, obs);

  // transform the information model back into state space for comparison
  mmInfo.convertStateVecIntoStateSpace();
  mmInfo._cov = mmInfo._cov.inverse().value();

  expectSamePosterior(mmInfo, mmKalman, 1e-8, 1e-8);
}

TEST(MeasurementUpdate, update_ComposedPositionVelocity__EqualsRepeatedSingleUpdates) // NOLINT
{
  auto mmComposed = makePrior<MMFull>();
  auto mmRepeated = makePrior<MMFull>();
  // clang-format off
  const auto pos = PosFull::FromLists({10.6, 5.3}, {
    {1.0, 0.0},
    {0.0, 0.5}
  });
  const auto vel = VelFull::FromLists({2.2, 0.8}, {
    {0.2, 0.0},
    {0.0, 0.2}
  });
  // clang-format on
  const KalmanFull filter{};

  // composed joint update vs. two consecutive single updates: identical for linear models
  mmComposed.update(filter, pos, vel);
  mmRepeated.update(filter, pos);
  mmRepeated.update(filter, vel);

  expectSamePosterior(mmComposed, mmRepeated, 1e-9, 1e-9);
}

TEST(MeasurementUpdate, update_ComposedFactoredVsComposedFull__SamePosterior) // NOLINT
{
  auto mmFull = makePrior<MMFull>();
  auto mmFact = makePrior<MMFact>();
  // clang-format off
  const auto posFull = PosFull::FromLists({10.6, 5.3}, {{1.0, 0.0}, {0.0, 0.5}});
  const auto posFact = PosFact::FromLists({10.6, 5.3}, {{1.0, 0.0}, {0.0, 0.5}});
  const auto velFull = VelFull::FromLists({2.2, 0.8}, {{0.2, 0.0}, {0.0, 0.2}});
  const auto velFact = VelFact::FromLists({2.2, 0.8}, {{0.2, 0.0}, {0.0, 0.2}});
  // clang-format on

  mmFull.update(KalmanFull{}, posFull, velFull);
  mmFact.update(KalmanFact{}, posFact, velFact);

  expectSamePosterior(mmFact, mmFull, 1e-9, 1e-9);
}

TEST(MeasurementUpdate, predictUpdateLoop__ConvergesToGroundTruth) // NOLINT
{
  using EgoMotionInst = tracking::env::EgoMotion<FullPolicy>;

  const auto           dt        = static_cast<Testvalue_type>(0.1);
  const auto           motion    = typename EgoMotionInst::InertialMotion{};
  const auto           geometry  = typename EgoMotionInst::Geometry{};
  const auto           egoMotion = EgoMotionInst(motion, geometry, dt);
  const KalmanFull     filter{};
  const Testvalue_type vxTrue = 1.0;
  const Testvalue_type vyTrue = 0.5;

  // heavily offset initial estimate with large uncertainty
  // clang-format off
  auto mm = MMFull::FromLists({5, 0, -5, 0}, {
    {100.0,  0.0,   0.0,  0.0},
    {  0.0, 10.0,   0.0,  0.0},
    {  0.0,  0.0, 100.0,  0.0},
    {  0.0,  0.0,   0.0, 10.0}
  });
  const auto R = PosFull::MeasurementCovFromList({
    {0.25, 0.00},
    {0.00, 0.25}
  });
  // clang-format on

  const int steps = 50;
  for (auto k = 1; k <= steps; ++k)
  {
    mm.predict(dt, filter, egoMotion);

    // noise-free position measurement of the constant velocity ground truth track
    typename PosFull::MeasurementVec z{};
    z.at_unsafe(PosFull::MEAS_X) = vxTrue * static_cast<Testvalue_type>(k) * dt;
    z.at_unsafe(PosFull::MEAS_Y) = vyTrue * static_cast<Testvalue_type>(k) * dt;
    const PosFull obs{z, R};

    mm.update(filter, obs);
  }

  const Testvalue_type tEnd = static_cast<Testvalue_type>(steps) * dt;
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::X), vxTrue * tEnd, 0.05);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::Y), vyTrue * tEnd, 0.05);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::VX), vxTrue, 0.2);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::VY), vyTrue, 0.2);

  // the covariance must have contracted in every observed direction
  EXPECT_LT(mm._cov.at_unsafe(StateDefCV::X, StateDefCV::X), 1.0);
  EXPECT_LT(mm._cov.at_unsafe(StateDefCV::Y, StateDefCV::Y), 1.0);
}
