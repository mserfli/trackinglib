#include <gtest/gtest.h>
#include "trackingLib/base/atomic_types.h"
#include "trackingLib/math/linalg/diagonal_matrix.hpp"
#include "trackingLib/math/linalg/errors.h"
#include "trackingLib/math/linalg/matrix.h"
#include "trackingLib/math/linalg/square_matrix.h"
#include "trackingLib/math/linalg/square_matrix.hpp" // IWYU pragma: keep -- Required template definitions.
#include "trackingLib/math/linalg/triangular_matrix.h"
#include "trackingLib/math/linalg/triangular_matrix.hpp"
#include "trackingLib/math/linalg/vector.h"
#include "trackingLib/math/linalg/vector.hpp" // IWYU pragma: keep -- Required template definitions.
#include <initializer_list>
#include <type_traits>
#include <utility>

using tracking::math::DiagonalMatrix;
using tracking::math::Errors;
using tracking::math::Matrix;
using tracking::math::SquareMatrix;
using tracking::math::TriangularMatrix;
using tracking::math::Vector;

static_assert(std::is_same_v<std::underlying_type_t<Errors>, int>, "Preserve the original enum representation");
static_assert(static_cast<int>(Errors::invalid_access_row) == 0);
static_assert(static_cast<int>(Errors::invalid_access_col) == 1);
static_assert(static_cast<int>(Errors::invalid_access_idx) == 2);
static_assert(static_cast<int>(Errors::divide_by_zero) == 3);
static_assert(static_cast<int>(Errors::not_integer_division) == 4);
static_assert(static_cast<int>(Errors::integer_divide_overflows) == 5);
static_assert(static_cast<int>(Errors::matrix_not_symmetric) == 6);
static_assert(static_cast<int>(Errors::matrix_not_positive_definite) == 7);

template <typename T>
class CheckedPrimitiveFactories: public ::testing::Test
{
};
using PrimitiveTypes = ::testing::Types<sint32, float32, float64>;
TYPED_TEST_SUITE(CheckedPrimitiveFactories, PrimitiveTypes);

TYPED_TEST(CheckedPrimitiveFactories, FromList__PreservesValuesAndExtraction) // NOLINT
{
  using T    = TypeParam;
  using List = std::initializer_list<std::initializer_list<T>>;
  using Flat = std::initializer_list<T>;
  static_assert(noexcept(Matrix<T, 2, 2, true>::FromList(std::declval<const List&>())));
  static_assert(noexcept(Matrix<T, 2, 2, false>::FromList(std::declval<const List&>())));
  static_assert(noexcept(SquareMatrix<T, 2, true>::FromList(std::declval<const List&>())));
  static_assert(noexcept(SquareMatrix<T, 2, false>::FromList(std::declval<const List&>())));
  static_assert(noexcept(Vector<T, 2>::FromList(std::declval<const Flat&>())));
  static_assert(noexcept(DiagonalMatrix<T, 2>::FromList(std::declval<const Flat&>())));
  static_assert(noexcept(DiagonalMatrix<T, 2>::FromList(std::declval<const List&>())));
  static_assert(noexcept(TriangularMatrix<T, 2, true, true>::FromList(std::declval<const List&>())));
  static_assert(noexcept(TriangularMatrix<T, 2, true, false>::FromList(std::declval<const List&>())));
  static_assert(noexcept(TriangularMatrix<T, 2, false, true>::FromList(std::declval<const List&>())));
  static_assert(noexcept(TriangularMatrix<T, 2, false, false>::FromList(std::declval<const List&>())));
  const List list{{1, 2}, {3, 4}};
  const auto row    = Matrix<T, 2, 2, true>::FromList(list);
  const auto col    = Matrix<T, 2, 2, false>::FromList(list);
  const auto square = SquareMatrix<T, 2, false>::FromList(list);
  const auto lower  = TriangularMatrix<T, 2, true, false>::FromList(list);
  const auto upper  = TriangularMatrix<T, 2, false, true>::FromList(list);
  const auto diag   = DiagonalMatrix<T, 2>::FromList(list);
  const auto flat   = DiagonalMatrix<T, 2>::FromList(Flat{1, 4});
  const auto vec    = Vector<T, 2>::FromList({1, 4});
  ASSERT_TRUE(row.has_value());
  ASSERT_TRUE(col.has_value());
  ASSERT_TRUE(square.has_value());
  ASSERT_TRUE(lower.has_value());
  ASSERT_TRUE(upper.has_value());
  ASSERT_TRUE(diag.has_value());
  ASSERT_TRUE(flat.has_value());
  ASSERT_TRUE(vec.has_value());
  for (sint32 r = 0; r < 2; ++r)
  {
    for (sint32 c = 0; c < 2; ++c)
    {
      EXPECT_EQ(row.value().at_unsafe(r, c), static_cast<T>(1 + r * 2 + c));
      EXPECT_EQ(col.value().at_unsafe(r, c), row.value().at_unsafe(r, c));
      EXPECT_EQ(square.value().at_unsafe(r, c), row.value().at_unsafe(r, c));
      EXPECT_EQ((static_cast<const SquareMatrix<T, 2, false>&>(lower.value()).at_unsafe(r, c)),
                c <= r ? row.value().at_unsafe(r, c) : T{0});
      EXPECT_EQ((static_cast<const SquareMatrix<T, 2, true>&>(upper.value()).at_unsafe(r, c)),
                c >= r ? row.value().at_unsafe(r, c) : T{0});
    }
    EXPECT_EQ(diag.value().at_unsafe(r), flat.value().at_unsafe(r));
    EXPECT_EQ(vec.value().at_unsafe(r), flat.value().at_unsafe(r));
  }
}

TYPED_TEST(CheckedPrimitiveFactories, FromList__RejectsMalformedDimensions) // NOLINT
{
  using T    = TypeParam;
  using List = std::initializer_list<std::initializer_list<T>>;
  using Flat = std::initializer_list<T>;
  for (const auto& list : {List{}, List{{1, 2}}, List{{1, 2}, {3, 4}, {5, 6}}})
  {
    const auto row    = Matrix<T, 2, 2, true>::FromList(list);
    const auto col    = Matrix<T, 2, 2, false>::FromList(list);
    const auto square = SquareMatrix<T, 2>::FromList(list);
    const auto tri    = TriangularMatrix<T, 2, false>::FromList(list);
    const auto diag   = DiagonalMatrix<T, 2>::FromList(list);
    ASSERT_FALSE(row.has_value());
    ASSERT_FALSE(col.has_value());
    ASSERT_FALSE(square.has_value());
    ASSERT_FALSE(tri.has_value());
    ASSERT_FALSE(diag.has_value());
    EXPECT_EQ(row.error(), Errors::invalid_initializer_list_row_count);
    EXPECT_EQ(col.error(), row.error());
    EXPECT_EQ(square.error(), row.error());
    EXPECT_EQ(tri.error(), row.error());
    EXPECT_EQ(diag.error(), row.error());
  }
  for (const auto& list : {List{{}, {}}, List{{1}, {2}}, List{{1, 2}, {3}}, List{{1, 2}, {3, 4, 5}}})
  {
    const auto row    = Matrix<T, 2, 2, true>::FromList(list);
    const auto col    = Matrix<T, 2, 2, false>::FromList(list);
    const auto square = SquareMatrix<T, 2>::FromList(list);
    const auto tri    = TriangularMatrix<T, 2, true>::FromList(list);
    const auto diag   = DiagonalMatrix<T, 2>::FromList(list);
    ASSERT_FALSE(row.has_value());
    ASSERT_FALSE(col.has_value());
    ASSERT_FALSE(square.has_value());
    ASSERT_FALSE(tri.has_value());
    ASSERT_FALSE(diag.has_value());
    EXPECT_EQ(row.error(), Errors::invalid_initializer_list_col_count);
    EXPECT_EQ(col.error(), row.error());
    EXPECT_EQ(square.error(), row.error());
    EXPECT_EQ(tri.error(), row.error());
    EXPECT_EQ(diag.error(), row.error());
  }
  for (const auto& list : {Flat{}, Flat{1}, Flat{1, 2, 3}})
  {
    const auto vec  = Vector<T, 2>::FromList(list);
    const auto diag = DiagonalMatrix<T, 2>::FromList(list);
    ASSERT_FALSE(vec.has_value());
    ASSERT_FALSE(diag.has_value());
    EXPECT_EQ(vec.error(), Errors::invalid_initializer_list_size);
    EXPECT_EQ(diag.error(), vec.error());
  }
}
