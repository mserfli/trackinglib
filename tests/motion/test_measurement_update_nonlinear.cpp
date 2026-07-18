#include <gtest/gtest.h>

#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include "trackingLib/observation/position_observation_model.h"
#include "trackingLib/observation/range_bearing_doppler_observation_model.h"
#include "trackingLib/observation/range_bearing_observation_model.h"
#include <cmath>

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
using StateDefCV     = tracking::motion::StateDefCV;

using MMFull  = tracking::motion::MotionModelCV<FullPolicy>;
using MMFact  = tracking::motion::MotionModelCV<FactoredPolicy>;
using RbFull  = tracking::observation::RangeBearingObservationModel<FullPolicy, StateDefCV>;
using RbFact  = tracking::observation::RangeBearingObservationModel<FactoredPolicy, StateDefCV>;
using RbdFull = tracking::observation::RangeBearingDopplerObservationModel<FullPolicy, StateDefCV>;
using PosFull = tracking::observation::PositionObservationModel<FullPolicy, StateDefCV>;
using PosFact = tracking::observation::PositionObservationModel<FactoredPolicy, StateDefCV>;

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

/// \brief Range/bearing measurement close to the prior of makePrior() (range ~11.18, bearing ~0.46)
template <typename RbModel_>
auto makeRangeBearingObs() -> RbModel_
{
  // clang-format off
  return RbModel_::FromLists({11.3, 0.50}, {
    {0.04,   0.0},
    {0.0, 0.0025}
  });
  // clang-format on
}

} // namespace

TEST(MeasurementUpdateNonlinear, update_KalmanFullRangeBearing__MatchesManualEkfReference) // NOLINT
{
  auto       mm  = makePrior<MMFull>();
  const auto obs = makeRangeBearingObs<RbFull>();

  // manual EKF reference composed with plain matrix operations at the prior state
  const auto x0 = mm._vec;
  const auto P0 = mm._cov;

  typename RbFull::JacobianMatrix H{};
  obs.computeJacobian(H, x0);
  const auto nu = obs.computeInnovation(obs.getVec(), obs.predictMeasurement(x0));

  const tracking::math::Matrix<Testvalue_type, 4, 2>    PHt{P0 * H.transpose()};
  const tracking::math::SquareMatrix<Testvalue_type, 2> S{(H * PHt) + obs.getCov()()};
  const auto                                            Sinv = S.inverse();
  const tracking::math::Matrix<Testvalue_type, 4, 2>    K{PHt * Sinv};

  auto xExp = x0;
  xExp += K * nu;
  const tracking::math::SquareMatrix<Testvalue_type, 4> IKH{tracking::math::SquareMatrix<Testvalue_type, 4>::Identity() -
                                                            (K * H)};
  const tracking::math::SquareMatrix<Testvalue_type, 4> PExp{(IKH * P0() * IKH.transpose()) +
                                                             ((K * obs.getCov()()) * K.transpose())};

  // call UUT
  mm.update(KalmanFull{}, obs);

  for (auto row = 0; row < StateDefCV::NUM_STATE_VARIABLES; ++row)
  {
    EXPECT_NEAR(mm._vec.at_unsafe(row), xExp.at_unsafe(row), 1e-9) << "vec row " << row;
    for (auto col = 0; col < StateDefCV::NUM_STATE_VARIABLES; ++col)
    {
      EXPECT_NEAR(mm._cov.at_unsafe(row, col), PExp.at_unsafe(row, col), 1e-9) << "cov row " << row << " col " << col;
    }
  }
}

TEST(MeasurementUpdateNonlinear, update_InformationVsKalman_RangeBearing__SamePosterior) // NOLINT
{
  auto       mmKalman = makePrior<MMFull>();
  auto       mmInfo   = makePrior<MMFull>();
  const auto obs      = makeRangeBearingObs<RbFull>();

  // transform the information model into information space: Y = inv(P), y = Y*x
  mmInfo._cov = mmInfo._cov.inverse().value();
  mmInfo._vec = static_cast<typename MMFull::StateVec>(mmInfo._cov() * mmInfo._vec);

  mmKalman.update(KalmanFull{}, obs);
  mmInfo.update(InformationFull{}, obs);

  // transform the information model back into state space for comparison
  mmInfo.convertStateVecIntoStateSpace();
  mmInfo._cov = mmInfo._cov.inverse().value();

  // exercises the effective measurement z_eff = nu + H*x of the linearized nonlinear model
  expectSamePosterior(mmInfo, mmKalman, 1e-8, 1e-8);
}

TEST(MeasurementUpdateNonlinear, update_KalmanFactoredVsFull_RangeBearing__SamePosterior) // NOLINT
{
  auto       mmFull  = makePrior<MMFull>();
  auto       mmFact  = makePrior<MMFact>();
  const auto obsFull = makeRangeBearingObs<RbFull>();
  const auto obsFact = makeRangeBearingObs<RbFact>();

  mmFull.update(KalmanFull{}, obsFull);
  mmFact.update(KalmanFact{}, obsFact);

  expectSamePosterior(mmFact, mmFull, 1e-9, 1e-9);
}

TEST(MeasurementUpdateNonlinear, update_InformationFactoredVsKalmanFactored_RangeBearing__SamePosterior) // NOLINT
{
  auto       mmKalman = makePrior<MMFact>();
  auto       mmInfo   = makePrior<MMFact>();
  const auto obs      = makeRangeBearingObs<RbFact>();

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

TEST(MeasurementUpdateNonlinear, update_KalmanFullSequentialVsBlock_RangeBearing__SamePosterior) // NOLINT
{
  auto             mmBlock      = makePrior<MMFull>();
  auto             mmSequential = makePrior<MMFull>();
  const auto       obs          = makeRangeBearingObs<RbFull>();
  const KalmanFull filter{};

  // H is evaluated once at the prior and the sequential path re-linearizes later innovations
  // around the entry state, so the equality with the block update also holds for nonlinear models
  mmBlock.update<update_mode::Block>(filter, obs);
  mmSequential.update<update_mode::Sequential>(filter, obs);

  expectSamePosterior(mmSequential, mmBlock, 1e-10, 1e-10);
}

TEST(MeasurementUpdateNonlinear, update_BearingWrapAcrossPi__CorrectionTakesShortPath) // NOLINT
{
  const Testvalue_type pi = std::acos(-1.0);

  // prior close to the -x axis: predicted bearing ~ +pi - 0.05
  // clang-format off
  auto mm = MMFull::FromLists({-10.0, 0.0, 0.5, 0.0}, {
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0}
  });
  // measured bearing just across the +-pi seam: the wrapped innovation is +0.08, the unwrapped
  // difference would be -6.20 and would yield a catastrophic correction
  const auto obs = RbFull::FromLists({10.0125, -pi + 0.03}, {
    {0.01,   0.0},
    {0.0, 0.0004}
  });
  // clang-format on

  mm.update(KalmanFull{}, obs);

  // wrapped small innovation: the position correction stays small (bearing gain * 0.08 rad at
  // range 10 is well below 1 m); the unwrapped innovation would displace the state by several 10 m
  const Testvalue_type dx = mm._vec.at_unsafe(StateDefCV::X) - (-10.0);
  const Testvalue_type dy = mm._vec.at_unsafe(StateDefCV::Y) - 0.5;
  EXPECT_LT(std::sqrt((dx * dx) + (dy * dy)), 1.0);

  // the posterior bearing moved from the prior (~pi - 0.05) toward the seam
  const Testvalue_type postBearing = std::atan2(mm._vec.at_unsafe(StateDefCV::Y), mm._vec.at_unsafe(StateDefCV::X));
  EXPECT_GT(std::abs(postBearing), pi - 0.05);
}

TEST(MeasurementUpdateNonlinear, update_ComposedRangeBearingPlusPosition_FactoredVsFull__SamePosterior) // NOLINT
{
  auto mmFull = makePrior<MMFull>();
  auto mmFact = makePrior<MMFact>();
  // clang-format off
  const auto rbFull  = makeRangeBearingObs<RbFull>();
  const auto rbFact  = makeRangeBearingObs<RbFact>();
  const auto posFull = PosFull::FromLists({10.4, 5.2}, {{1.0, 0.0}, {0.0, 0.5}});
  const auto posFact = PosFact::FromLists({10.4, 5.2}, {{1.0, 0.0}, {0.0, 0.5}});
  // clang-format on

  // composed nonlinear + linear stacking must agree across the covariance representations
  mmFull.update(KalmanFull{}, rbFull, posFull);
  mmFact.update(KalmanFact{}, rbFact, posFact);

  expectSamePosterior(mmFact, mmFull, 1e-9, 1e-9);
}

TEST(MeasurementUpdateNonlinear, predictUpdateLoop_RangeBearingDoppler__ConvergesToGroundTruth) // NOLINT
{
  using EgoMotionInst = tracking::env::EgoMotion<FullPolicy>;

  const auto           dt        = static_cast<Testvalue_type>(0.1);
  const auto           motion    = typename EgoMotionInst::InertialMotion{};
  const auto           geometry  = typename EgoMotionInst::Geometry{};
  const auto           egoMotion = EgoMotionInst(motion, geometry, dt);
  const KalmanFull     filter{};
  const Testvalue_type x0True = 2.0;
  const Testvalue_type y0True = 1.0;
  const Testvalue_type vxTrue = 1.0;
  const Testvalue_type vyTrue = 0.5;

  // heavily offset initial estimate with large uncertainty
  // clang-format off
  auto mm = MMFull::FromLists({5, 0, -3, 0}, {
    {100.0,  0.0,   0.0,  0.0},
    {  0.0, 10.0,   0.0,  0.0},
    {  0.0,  0.0, 100.0,  0.0},
    {  0.0,  0.0,   0.0, 10.0}
  });
  const auto R = RbdFull::MeasurementCovFromList({
    {0.04,   0.0,  0.0},
    {0.0, 0.0004,  0.0},
    {0.0,    0.0, 0.01}
  });
  // clang-format on

  const int steps = 60;
  for (auto k = 1; k <= steps; ++k)
  {
    mm.predict(dt, filter, egoMotion);

    // noise-free range/bearing/doppler measurement of the constant velocity ground truth track
    const Testvalue_type px    = x0True + (vxTrue * static_cast<Testvalue_type>(k) * dt);
    const Testvalue_type py    = y0True + (vyTrue * static_cast<Testvalue_type>(k) * dt);
    const Testvalue_type range = std::sqrt((px * px) + (py * py));

    typename RbdFull::MeasurementVec z{};
    z.at_unsafe(RbdFull::MEAS_RANGE)   = range;
    z.at_unsafe(RbdFull::MEAS_BEARING) = std::atan2(py, px);
    z.at_unsafe(RbdFull::MEAS_DOPPLER) = ((px * vxTrue) + (py * vyTrue)) / range;
    const RbdFull obs{z, R};

    mm.update(filter, obs);
  }

  const Testvalue_type tEnd = static_cast<Testvalue_type>(steps) * dt;
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::X), x0True + (vxTrue * tEnd), 0.1);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::Y), y0True + (vyTrue * tEnd), 0.1);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::VX), vxTrue, 0.2);
  EXPECT_NEAR(mm._vec.at_unsafe(StateDefCV::VY), vyTrue, 0.2);

  // the covariance must have contracted in every observed direction
  EXPECT_LT(mm._cov.at_unsafe(StateDefCV::X, StateDefCV::X), 1.0);
  EXPECT_LT(mm._cov.at_unsafe(StateDefCV::Y, StateDefCV::Y), 1.0);
  EXPECT_LT(mm._cov.at_unsafe(StateDefCV::VX, StateDefCV::VX), 1.0);
  EXPECT_LT(mm._cov.at_unsafe(StateDefCV::VY, StateDefCV::VY), 1.0);
}

TEST(MeasurementUpdateNonlinear, predictUpdateLoop_DopplerImprovesVelocity__SmallerVelocityVariance) // NOLINT
{
  using EgoMotionInst = tracking::env::EgoMotion<FullPolicy>;

  const auto           dt        = static_cast<Testvalue_type>(0.1);
  const auto           motion    = typename EgoMotionInst::InertialMotion{};
  const auto           geometry  = typename EgoMotionInst::Geometry{};
  const auto           egoMotion = EgoMotionInst(motion, geometry, dt);
  const KalmanFull     filter{};
  const Testvalue_type x0True = 2.0;
  const Testvalue_type y0True = 1.0;
  const Testvalue_type vxTrue = 1.0;
  const Testvalue_type vyTrue = 0.5;

  const auto makeInitial = []() {
    // clang-format off
    return MMFull::FromLists({5, 0, -3, 0}, {
      {100.0,  0.0,   0.0,  0.0},
      {  0.0, 10.0,   0.0,  0.0},
      {  0.0,  0.0, 100.0,  0.0},
      {  0.0,  0.0,   0.0, 10.0}
    });
    // clang-format on
  };
  auto mmRb  = makeInitial();
  auto mmRbd = makeInitial();

  // clang-format off
  const auto Rrb = RbFull::MeasurementCovFromList({
    {0.04,   0.0},
    {0.0, 0.0004}
  });
  const auto Rrbd = RbdFull::MeasurementCovFromList({
    {0.04,   0.0,  0.0},
    {0.0, 0.0004,  0.0},
    {0.0,    0.0, 0.01}
  });
  // clang-format on

  const int steps = 60;
  for (auto k = 1; k <= steps; ++k)
  {
    mmRb.predict(dt, filter, egoMotion);
    mmRbd.predict(dt, filter, egoMotion);

    const Testvalue_type px    = x0True + (vxTrue * static_cast<Testvalue_type>(k) * dt);
    const Testvalue_type py    = y0True + (vyTrue * static_cast<Testvalue_type>(k) * dt);
    const Testvalue_type range = std::sqrt((px * px) + (py * py));

    typename RbFull::MeasurementVec zRb{};
    zRb.at_unsafe(RbFull::MEAS_RANGE)   = range;
    zRb.at_unsafe(RbFull::MEAS_BEARING) = std::atan2(py, px);
    mmRb.update(filter, RbFull{zRb, Rrb});

    typename RbdFull::MeasurementVec zRbd{};
    zRbd.at_unsafe(RbdFull::MEAS_RANGE)   = range;
    zRbd.at_unsafe(RbdFull::MEAS_BEARING) = std::atan2(py, px);
    zRbd.at_unsafe(RbdFull::MEAS_DOPPLER) = ((px * vxTrue) + (py * vyTrue)) / range;
    mmRbd.update(filter, RbdFull{zRbd, Rrbd});
  }

  // the doppler measurement adds independent radial velocity information every step:
  // the posterior velocity variances must be strictly smaller than without doppler
  EXPECT_LT(mmRbd._cov.at_unsafe(StateDefCV::VX, StateDefCV::VX), mmRb._cov.at_unsafe(StateDefCV::VX, StateDefCV::VX));
  EXPECT_LT(mmRbd._cov.at_unsafe(StateDefCV::VY, StateDefCV::VY), mmRb._cov.at_unsafe(StateDefCV::VY, StateDefCV::VY));
}
