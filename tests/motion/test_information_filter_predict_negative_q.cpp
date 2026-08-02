#include <gtest/gtest.h>

#include "trackingLib/filter/information_filter.hpp"              // IWYU pragma: keep
#include "trackingLib/math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/diagonal_matrix.hpp"            // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix.hpp"                     // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.hpp"              // IWYU pragma: keep
#include <cmath>

// InformationFilter<FactoredPolicy>::predictCovariance computes, per process-noise entry i,
// ci = -1 / (1/Q(i) + Gi'*Y*Gi). An invalid (negative or NaN) Q(i) can drive the denominator to
// (or through) zero, turning ci into +/-Inf or NaN, which then propagates into Y via rank1Update.
// A guard skips process-noise entries with a non-positive/NaN variance (Q(i) == 0 already resolves
// to no update via IEEE arithmetic and rank1Update's own c==0 early return; this guard's real
// target is negative/NaN Q(i), which has no such natural protection).

using ValueType_     = float64;
using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<ValueType_>;
using IF             = tracking::filter::InformationFilter<FactoredPolicy>;
using CovType        = FactoredPolicy::Instantiate<2>;

TEST(InformationFilterPredict, predictCovariance_FactoredNegativeQ__StaysFiniteAndPSD) // NOLINT
{
  auto Y = CovType::Identity();

  const auto A = tracking::math::SquareMatrix<ValueType_, 2>::Identity();
  const auto G = tracking::math::Matrix<ValueType_, 2, 2>::FromList({{1.0, 0.0}, {0.0, 1.0}});
  // clang-format off
  const auto Q = tracking::math::DiagonalMatrix<ValueType_, 2>::FromList({1.0, -1.0});
  // clang-format on

  IF::predictCovariance<2, 2>(Y, A, G, Q);

  const auto composed = Y();
  for (auto row = 0; row < 2; ++row)
  {
    for (auto col = 0; col < 2; ++col)
    {
      EXPECT_TRUE(std::isfinite(composed.at_unsafe(row, col))) << "row=" << row << " col=" << col;
    }
  }
  EXPECT_TRUE(Y.isPositiveSemiDefinite());
}
