#include <gtest/gtest.h>

#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include <algorithm>                              // for std::max
#include <cmath>                                  // for std::abs

// The InformationFilter's non-factored covariance prediction only
// commits the predicted information matrix Y when SquareMatrix::isPositiveSemiDefinite() is true
// (information_filter.hpp / generic_predict.hpp guards). Because that check forwards to the strict
// Cholesky pivot, a legitimately singular PSD predicted Y (an unobserved subspace -> zero
// eigenvalue) is rejected and Y is FROZEN. The RED test drives exactly that case and asserts the
// intended transition still applies (Y must change) -> it FAILS on this branch, confirming the
// freeze. The strictly-PD control is a regression guard so the eventual fix does not over-loosen
// the "zero-Y no-information" protection.

using Testvalue_type = float64;
using FullPolicy     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
using MM             = tracking::motion::MotionModelCV<FullPolicy>;
using EgoMotionInst  = tracking::env::EgoMotion<FullPolicy>;
using IF             = tracking::filter::InformationFilter<FullPolicy>;

// instantiate for full coverage report
template class tracking::motion::MotionModelCV<FullPolicy>;

namespace
{

// no-ego-motion instance: displacement is exactly zero, so generic::Predict takes the plain
// filter.predictCovariance(Y, A, G, Q) information-space branch (no ego compensation)
auto makeNoEgoMotion() -> EgoMotionInst
{
  return EgoMotionInst(EgoMotionInst::InertialMotion{}, EgoMotionInst::Geometry{}, static_cast<Testvalue_type>(0.1));
}

/// \brief Largest absolute element-wise difference between two information matrices
auto maxAbsDiff(const MM::StateMatrix& a, const MM::StateMatrix& b) -> Testvalue_type
{
  Testvalue_type worst = 0.0;
  for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
  {
    for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
    {
      worst = std::max(worst, std::abs(a.at_unsafe(row, col) - b.at_unsafe(row, col)));
    }
  }
  return worst;
}

} // namespace

TEST(InformationFilterPredict, predict_RankDeficientInformationMatrix__DoesNotFreeze) // NOLINT
{
  const auto dt        = static_cast<Testvalue_type>(0.1);
  const auto egoMotion = makeNoEgoMotion();
  IF         filter{};

  // Information matrix with the velocity subspace unobserved (zero information) -> singular PSD,
  // i.e. an exactly-zero eigenvalue. State order is {X, VX, Y, VY}.
  // clang-format off
  const auto Ysingular = MM::StateCovFromList({
    {4.0, 0.0, 0.0, 0.0},   // X  observed
    {0.0, 0.0, 0.0, 0.0},   // VX unobserved
    {0.0, 0.0, 4.0, 0.0},   // Y  observed
    {0.0, 0.0, 0.0, 0.0}    // VY unobserved
  });
  // clang-format on
  const auto ySingular = MM::StateVecFromList({8.0, 0.0, 8.0, 0.0});

  // Construct with a PD placeholder to satisfy the ctor's determinant assert, then install the
  // singular PSD information state through the internal-use accessors.
  MM mm{MM::StateVecFromList({0, 0, 0, 0}), MM::StateCovFromList({{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}})};
  mm.getCovForInternalUse() = Ysingular;
  mm.getVecForInternalUse() = ySingular;

  const MM::StateMatrix before = mm.getCov()();

  mm.predict(dt, filter, egoMotion);

  const MM::StateMatrix after = mm.getCov()();

  // The observed-subspace information genuinely transitions under the CV model, so a correct
  // (semi-definite-aware) guard would commit a changed Y. On this branch the strict PSD check
  // rejects the valid singular PSD predicted Y and freezes it -> this assertion FAILS (RED).
  EXPECT_GT(maxAbsDiff(before, after), static_cast<Testvalue_type>(1e-9))
      << "information matrix was frozen (unchanged) across the predict";
}

TEST(InformationFilterPredict, predict_StrictlyPositiveDefiniteInformationMatrix__Updates) // NOLINT
{
  const auto dt        = static_cast<Testvalue_type>(0.1);
  const auto egoMotion = makeNoEgoMotion();
  IF         filter{};

  // strictly PD information matrix -> the predict must update it normally (regression guard: the
  // eventual fix must keep updating PD matrices and must NOT resurrect a genuinely zero-Y state)
  // clang-format off
  const auto Ypd = MM::StateCovFromList({
    {5.0, 0.0, 0.0, 0.0},
    {0.0, 2.0, 0.0, 0.0},
    {0.0, 0.0, 5.0, 0.0},
    {0.0, 0.0, 0.0, 2.0}
  });
  // clang-format on
  const auto yPd = MM::StateVecFromList({10.0, 4.0, 10.0, 4.0});

  MM mm{yPd, Ypd};

  const MM::StateMatrix before = mm.getCov()();

  mm.predict(dt, filter, egoMotion);

  const MM::StateMatrix after = mm.getCov()();

  EXPECT_GT(maxAbsDiff(before, after), static_cast<Testvalue_type>(1e-9))
      << "strictly PD information matrix should be updated by the predict";
}
