#include <gtest/gtest.h>
#include "base/atomic_types.h"
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/errors.h"
#include "trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp"
#include <initializer_list>
#include <limits>
#include <utility>

using tracking::math::CovarianceMatrixFactored;
using tracking::math::CovarianceMatrixFull;
using tracking::math::Errors;
namespace conversions = tracking::math::conversions;

template <typename T>
class CheckedCovarianceFactories: public ::testing::Test
{
};
using CovarianceTypes = ::testing::Types<float32, float64>;
TYPED_TEST_SUITE(CheckedCovarianceFactories, CovarianceTypes);

TYPED_TEST(CheckedCovarianceFactories, FromList__ChecksSymmetryAndPreservesClamp) // NOLINT
{
  using T        = TypeParam;
  using Full     = CovarianceMatrixFull<T, 2>;
  using Factored = CovarianceMatrixFactored<T, 2>;
  using List     = std::initializer_list<std::initializer_list<T>>;
  using Flat     = std::initializer_list<T>;
  static_assert(noexcept(Full::FromList(std::declval<const List&>())));
  static_assert(noexcept(Factored::FromList(std::declval<const List&>(), std::declval<const Flat&>())));
  static_assert(noexcept(conversions::CovarianceMatrixFactoredFromList<T, 2>(std::declval<const List&>())));
  static_assert(noexcept(conversions::CovarianceMatrixFactoredFromCovarianceMatrixFull<T, 2>(std::declval<const Full&>())));
  const List asymmetric{{1, 2}, {0, 1}};
  const auto fullError     = Full::FromList(asymmetric);
  const auto factoredError = conversions::CovarianceMatrixFactoredFromList<T, 2>(asymmetric);
  ASSERT_FALSE(fullError.has_value());
  ASSERT_FALSE(factoredError.has_value());
  EXPECT_EQ(fullError.error(), Errors::matrix_not_symmetric);
  EXPECT_EQ(factoredError.error(), fullError.error());

  const auto singular = Full::FromList({{0, 0}, {0, 0}});
  ASSERT_TRUE(singular.has_value());
  EXPECT_EQ(singular.value().at_unsafe(0, 0), T{0});

  // Full construction checks symmetry only; factored conversion retains the UDU pivot clamp.
  const List indefinite{{-1, 0}, {0, 1}};
  const auto full     = Full::FromList(indefinite);
  const auto factored = conversions::CovarianceMatrixFactoredFromList<T, 2>(indefinite);
  ASSERT_TRUE(full.has_value());
  ASSERT_TRUE(factored.has_value());
  EXPECT_FALSE(full.value().isPositiveDefinite());
  EXPECT_TRUE(factored.value().isPositiveDefinite());
  EXPECT_EQ(full.value().at_unsafe(0, 0), T{-1});
  EXPECT_EQ(factored.value().D().at_unsafe(0), std::numeric_limits<T>::epsilon());

  const auto valid = Factored::FromList({{1, 2}, {99, 1}}, {3, 4});
  ASSERT_TRUE(valid.has_value());
  EXPECT_EQ(valid.value().U().at_unsafe(0, 1), T{2});
  EXPECT_EQ(valid.value().D().at_unsafe(1), T{4});
}

TYPED_TEST(CheckedCovarianceFactories, FromList__ReportsDimensionsBeforeInvariants) // NOLINT
{
  using T        = TypeParam;
  using Factored = CovarianceMatrixFactored<T, 2>;
  using List     = std::initializer_list<std::initializer_list<T>>;
  using Flat     = std::initializer_list<T>;
  for (const auto& list : {List{}, List{{1, 0}}, List{{1, 0}, {0, 1}, {0, 0}}})
  {
    const auto result    = Factored::FromList(list, {});
    const auto full      = CovarianceMatrixFull<T, 2>::FromList(list);
    const auto converted = conversions::CovarianceMatrixFactoredFromList<T, 2>(list);
    ASSERT_FALSE(result.has_value());
    ASSERT_FALSE(full.has_value());
    ASSERT_FALSE(converted.has_value());
    EXPECT_EQ(result.error(), Errors::invalid_initializer_list_row_count);
    EXPECT_EQ(full.error(), result.error());
    EXPECT_EQ(converted.error(), result.error());
  }
  for (const auto& list : {List{{}, {}}, List{{1}, {1}}, List{{2, 0}, {1}}, List{{1, 0}, {1, 2, 3}}})
  {
    const auto result    = Factored::FromList(list, {});
    const auto full      = CovarianceMatrixFull<T, 2>::FromList(list);
    const auto converted = conversions::CovarianceMatrixFactoredFromList<T, 2>(list);
    ASSERT_FALSE(result.has_value());
    ASSERT_FALSE(full.has_value());
    ASSERT_FALSE(converted.has_value());
    EXPECT_EQ(result.error(), Errors::invalid_initializer_list_col_count);
    EXPECT_EQ(full.error(), result.error());
    EXPECT_EQ(converted.error(), result.error());
  }
  for (const auto& list : {Flat{}, Flat{1}, Flat{1, 2, 3}})
  {
    const auto result = Factored::FromList({{2, 0}, {0, 1}}, list);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), Errors::invalid_initializer_list_size);
  }
  const auto badUpper = Factored::FromList({{2, 0}, {0, 1}}, {0, -1});
  ASSERT_FALSE(badUpper.has_value());
  EXPECT_EQ(badUpper.error(), Errors::matrix_not_unit_upper_triangular);
  for (const auto& list : {Flat{0, 1}, Flat{1, 0}, Flat{-1, 1}, Flat{1, -1}})
  {
    const auto result = Factored::FromList({{1, 0}, {99, 1}}, list);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), Errors::matrix_not_positive_definite);
  }
}
