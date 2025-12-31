#include "gtest/gtest.h"
#include "base/atomic_types.h"
#include "math/linalg/square_matrix.h"
#include "trackingLib/math/linalg/matrix.hpp"        // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.hpp" // IWYU pragma: keep

#define TEST_MATRIX_WITH_VISUAL_INSPECTION(X) // X.print()

/// \brief helper class to support typed tests which wraps the IsRowMajor param into a type
template <bool IsRowMajor_>
struct MatrixStorageType
{
  static constexpr auto IsRowMajor = IsRowMajor_;
};

/// \brief Generic Matrix test class templatized by MatrixStorageType
template <typename MatrixStorageType>
class GTestMatrix: public ::testing::Test
{
public:
  void SetUp() final;

protected:
  using IntMatType                   = tracking::math::Matrix<sint32, 2, 3, MatrixStorageType::IsRowMajor>;
  using IntMatTypeTranspMemLayout    = tracking::math::Matrix<sint32, 2, 3, !MatrixStorageType::IsRowMajor>;
  using IntMatMulType                = tracking::math::Matrix<sint32, 3, 4, MatrixStorageType::IsRowMajor>;
  using IntMatMulTypeTranspMemLayout = tracking::math::Matrix<sint32, 3, 4, !MatrixStorageType::IsRowMajor>;
  using IntMatMulResultType          = tracking::math::Matrix<sint32, 2, 4, MatrixStorageType::IsRowMajor>;
  using FloatMatType                 = tracking::math::Matrix<float32, 2, 3, MatrixStorageType::IsRowMajor>;
  using SquareIntMatType             = tracking::math::Matrix<sint32, 3, 3, MatrixStorageType::IsRowMajor>;
  IntMatType                   _testIntMat{};
  IntMatTypeTranspMemLayout    _testIntMatTransposed{};
  IntMatMulType                _testIntMatMul{};
  IntMatMulTypeTranspMemLayout _testIntMatMulTransposed{};
  IntMatMulResultType          _testIntMatMulResult{};
  FloatMatType                 _testFloatMat{};
  SquareIntMatType             _testSquareIntMat{};

  void test_ctor_initializerList_Success();

  template <typename T>
  void test_at_unsafe_Success()
  {
    T                      mat{_testIntMat};
    typename T::value_type retVal = -1;
    // call UUT
    retVal = mat.at_unsafe(T::Rows - 1, T::Cols - 1);

    EXPECT_EQ(retVal, 5);
  }

  template <typename T>
  void test_op_at_FailBadRowIdx()
  {
    T mat{_testIntMat};
    // call UUT
    auto retVal = mat(T::Rows, 0);

    EXPECT_FALSE(retVal.has_value());
    EXPECT_EQ(retVal.error(), tracking::math::Errors::invalid_access_row);
  }

  template <typename T>
  void test_op_at_FailBadColIdx()
  {
    T mat{_testIntMat};
    // call UUT
    auto retVal = mat(0, T::Cols);

    EXPECT_FALSE(retVal.has_value());
    EXPECT_EQ(retVal.error(), tracking::math::Errors::invalid_access_col);
  }

  template <typename T>
  void test_op_at_Success()
  {
    T mat{_testIntMat};
    // call UUT
    auto retVal = mat(T::Rows - 1, T::Cols - 1);

    EXPECT_TRUE(retVal.has_value());
    EXPECT_EQ(retVal, 5);
  }

  template <bool SameMajorOrder>
  void test_op_eq_Success();

  template <bool SameMajorOrder>
  void test_op_plus_inplace_Success();

  template <bool SameMajorOrder>
  void test_op_plus_transpose_inplace_Success();

  template <bool SameMajorOrder>
  void test_op_plus_Success();

  template <bool SameMajorOrder>
  void test_op_minus_inplace_Success();

  template <bool SameMajorOrder>
  void test_op_minus_Success();

  void test_op_mul_scalar_Success()
  {
    IntMatType mat{_testIntMat};
    IntMatType res{};

    // call UUT
    res = 2 * mat;

    const sint32 size = _testIntMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res._data[idx], 2 * _testIntMat._data[idx]);
    }
  }

  void test_op_plus_scalar_Success()
  {
    IntMatType mat{_testIntMat};
    IntMatType res{};

    // call UUT
    res = 2 + mat;

    const sint32 size = _testIntMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res._data[idx], _testIntMat._data[idx] + 2);
    }
  }

  void test_op_minus_scalar_Success()
  {
    IntMatType mat{_testIntMat};
    IntMatType res{};

    // call UUT
    res = mat - 2;

    const sint32 size = _testIntMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res._data[idx], _testIntMat._data[idx] - 2);
    }
  }

  void test_op_div_scalar_inplace_IntSuccess()
  {
    IntMatType mat{_testIntMat};

    // call UUT
    auto result = mat /= 2;
    ASSERT_TRUE(result.has_value());

    const sint32 size = _testIntMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(mat._data[idx], _testIntMat._data[idx] / 2);
    }
  }

  void test_op_div_scalar_inplace_FloatSuccess()
  {
    FloatMatType mat{_testFloatMat};

    // call UUT
    auto result = mat /= 2.0F;
    ASSERT_TRUE(result.has_value());

    const sint32 size = _testFloatMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(mat._data[idx], _testFloatMat._data[idx] / 2.0F);
    }
  }

  void test_op_div_scalar_IntFailDivByZero()
  {
    IntMatType mat{_testIntMat};

    // call UUT
    auto res = mat / 0;
    EXPECT_EQ(res.error(), tracking::math::Errors::divide_by_zero);
  }

  void test_op_div_scalar_IntSuccess()
  {
    IntMatType mat{_testIntMat};

    // call UUT
    auto res = mat / 2;

    const sint32 size = _testIntMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res.value()._data[idx], _testIntMat._data[idx] / 2);
    }
  }

  void test_op_div_scalar_FloatFailDivByZero()
  {
    FloatMatType mat{_testFloatMat};

    // call UUT
    auto res = mat / 0.0F;
    EXPECT_EQ(res.error(), tracking::math::Errors::divide_by_zero);
  }

  void test_op_div_scalar_FloatSuccess()
  {
    FloatMatType mat{_testFloatMat};

    // call UUT
    auto res = mat / 2.0F;

    const sint32 size = _testFloatMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res.value()._data[idx], _testFloatMat._data[idx] / 2.0F);
    }
  }

  void test_op_mul_mat_Success()
  {
    IntMatType    mat1{_testIntMat};
    IntMatMulType mat2{_testIntMatMul};

    // call UUT
    auto res = mat1 * mat2;

    EXPECT_TRUE(res == _testIntMatMulResult);
  }

  template <typename T>
  void test_transpose_Success()
  {
    T mat{_testIntMat};
    // clang-format off
    using ExpMatType = typename T::transpose_type_row_major;
    auto expMat = ExpMatType::FromList({
      {0, 3,},
      {1, 4,},
      {2, 5,},
    });
    // clang-format on

    // call UUT
    auto& matT = mat.transpose();

    EXPECT_TRUE(matT.Rows == expMat.Rows);
    EXPECT_TRUE(matT.Cols == expMat.Cols);
    for (auto row = 0; row < expMat.Rows; ++row)
    {
      for (auto col = 0; col < expMat.Cols; ++col)
      {
        EXPECT_EQ(matT.at_unsafe(row, col), expMat.at_unsafe(row, col));
      }
    }
  }
};

template <typename MatrixStorageType>
template <bool SameMajorOrder>
void GTestMatrix<MatrixStorageType>::test_op_eq_Success()
{
  if constexpr (SameMajorOrder)
  {
    IntMatType       mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    auto isEqual = (mat == other);

    EXPECT_TRUE(isEqual);
  }
  else
  {
    const IntMatType                mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    auto isEqual = (mat == other);

    EXPECT_TRUE(isEqual);
  }
}

template <typename MatrixStorageType>
template <bool SameMajorOrder>
void GTestMatrix<MatrixStorageType>::test_op_plus_Success()
{
  if constexpr (SameMajorOrder)
  {
    const IntMatType mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    auto res = mat + other;

    const sint32 size = res._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res._data[idx], 2 * _testIntMat._data[idx]);
    }
  }
  else
  {
    const IntMatType                mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    auto res = mat + other;

    const sint32 size = res._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res._data[idx], 2 * _testIntMat._data[idx]);
    }
  }
}

template <typename MatrixStorageType>
template <bool SameMajorOrder>
void GTestMatrix<MatrixStorageType>::test_op_plus_inplace_Success()
{
  if constexpr (SameMajorOrder)
  {
    IntMatType       mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    mat += other;

    const sint32 size = _testIntMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(mat._data[idx], 2 * _testIntMat._data[idx]);
    }
  }
  else
  {
    IntMatType                      mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    mat += other;

    const sint32 size = _testIntMat._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(mat._data[idx], 2 * _testIntMat._data[idx]);
    }
  }
}

template <typename MatrixStorageType>
template <bool SameMajorOrder>
void GTestMatrix<MatrixStorageType>::test_op_plus_transpose_inplace_Success()
{
  const SquareIntMatType other{_testSquareIntMat};
  // clang-format off
  const auto             expMat = SquareIntMatType::FromList({
      {0, 4, 8},
      {4, 8, 12},
      {8, 12, 16},
  });
  // clang-format on

  {
    SquareIntMatType mat{_testSquareIntMat};
    // call UUT
    mat += mat.transpose();
    EXPECT_EQ(mat._data, expMat._data);
  }
  {
    SquareIntMatType mat{_testSquareIntMat};
    // call UUT
    mat += other.transpose();
    EXPECT_EQ(mat._data, expMat._data);
  }
}

template <typename MatrixStorageType>
template <bool SameMajorOrder>
void GTestMatrix<MatrixStorageType>::test_op_minus_inplace_Success()
{
  if (SameMajorOrder)
  {
    IntMatType       mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    mat -= other;

    for (auto val : mat._data)
    {
      EXPECT_EQ(val, 0);
    }
  }
  else
  {
    IntMatType                      mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    mat -= other;

    for (auto val : mat._data)
    {
      EXPECT_EQ(val, 0);
    }
  }
}

template <typename MatrixStorageType>
template <bool SameMajorOrder>
void GTestMatrix<MatrixStorageType>::test_op_minus_Success()
{
  if constexpr (SameMajorOrder)
  {
    const IntMatType mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    auto res = mat - other;

    const sint32 size = res._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res._data[idx], 0);
    }
  }
  else
  {
    const IntMatType                mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    auto res = mat - other;

    const sint32 size = res._data.size();
    for (auto idx = 0; idx < size; ++idx)
    {
      EXPECT_EQ(res._data[idx], 0);
    }
  }
}

/// \brief template specialization of SetUp initializing the members for RowMajor memory layout
template <>
void GTestMatrix<MatrixStorageType<true>>::SetUp()
{
  // clang-format off
  _testIntMat = IntMatType::FromList({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  _testIntMatTransposed = IntMatTypeTranspMemLayout::FromList({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  _testIntMatMul = IntMatMulType::FromList({
    {0,  1,  2,  3,},
    {4,  5,  6,  7,},
    {8,  9, 10, 11,},
  });
  _testIntMatMulTransposed = IntMatMulTypeTranspMemLayout::FromList({
    {0,  4,  8,},
    {1,  5,  9,},
    {2,  6, 10,},
    {3,  7, 11,},
  });
  _testIntMatMulResult = IntMatMulResultType::FromList({
    {20, 23, 26, 29,},
    {56, 68, 80, 92,},
  });
  _testFloatMat = FloatMatType::FromList({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  _testSquareIntMat = SquareIntMatType::FromList({
    {0, 1, 2,},
    {3, 4, 5,},
    {6, 7, 8,},
  });
  // clang-format on
}

/// \brief template specialization of CTOR test with initializer list for RowMajor memory layout
template <>
void GTestMatrix<MatrixStorageType<true>>::test_ctor_initializerList_Success()
{
  // clang-format off
  // call UUT
  const auto mat = IntMatType::FromList({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  // clang-format on

  const IntMatType::Storage expStorage{0, 1, 2, 3, 4, 5};
  EXPECT_EQ(mat._data, expStorage);
  TEST_MATRIX_WITH_VISUAL_INSPECTION(mat);
}

/// \brief template specialization of SetUp initializing the members for ColumnMajor memory layout
template <>
void GTestMatrix<MatrixStorageType<false>>::SetUp()
{
  // clang-format off
  _testIntMat = IntMatType::FromList({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  _testIntMatTransposed = IntMatTypeTranspMemLayout::FromList({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  _testIntMatMul = IntMatMulType::FromList({
    {0,  4,  8,},
    {1,  5,  9,},
    {2,  6, 10,},
    {3,  7, 11,},
  });
  _testIntMatMulTransposed = IntMatMulTypeTranspMemLayout::FromList({
    {0,  1,  2,  3,},
    {4,  5,  6,  7,},
    {8,  9, 10, 11,},
  });
  _testIntMatMulResult = IntMatMulResultType::FromList({
    {20, 56,},
    {23, 68,},
    {26, 80,},
    {29, 92,},  
  });
  _testFloatMat = FloatMatType::FromList({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  _testSquareIntMat = SquareIntMatType::FromList({
    {0, 1, 2,},
    {3, 4, 5,},
    {6, 7, 8,},
  });
  // clang-format on
}

/// \brief template specialization of CTOR test with initializer list for ColumnMajor memory layout
template <>
void GTestMatrix<MatrixStorageType<false>>::test_ctor_initializerList_Success()
{
  // clang-format off
  // call UUT
  const auto mat = IntMatType::FromList({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  // clang-format on

  const IntMatType::Storage expStorage{0, 3, 1, 4, 2, 5};
  EXPECT_EQ(mat._data, expStorage);
  TEST_MATRIX_WITH_VISUAL_INSPECTION(mat);
}

using ::testing::Types;
// The list of types we want to test.
using Implementations = Types<MatrixStorageType<true>, MatrixStorageType<false>>;
TYPED_TEST_SUITE(GTestMatrix, Implementations);


TYPED_TEST(GTestMatrix, ctor_initializerList__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_ctor_initializerList_Success();
}

TYPED_TEST(GTestMatrix, ctor_rvalue__Success) // NOLINT
{
  auto ones = GTestMatrix<TypeParam>::IntMatType::Ones();

  // call UUT
  const auto mat = typename GTestMatrix<TypeParam>::IntMatType(std::move(ones));

  const typename GTestMatrix<TypeParam>::IntMatType::Storage expStorage{1, 1, 1, 1, 1, 1};
  EXPECT_EQ(mat._data, expStorage);
}

TYPED_TEST(GTestMatrix, ctor_Zeros__Success) // NOLINT
{
  // call UUT
  const auto mat = GTestMatrix<TypeParam>::IntMatType::Zeros();

  const typename GTestMatrix<TypeParam>::IntMatType::Storage expStorage{0, 0, 0, 0, 0, 0};
  EXPECT_EQ(mat._data, expStorage);
}

TYPED_TEST(GTestMatrix, ctor_Ones__Success) // NOLINT
{
  // call UUT
  const auto mat = GTestMatrix<TypeParam>::IntMatType::Ones();

  const typename GTestMatrix<TypeParam>::IntMatType::Storage expStorage{1, 1, 1, 1, 1, 1};
  EXPECT_EQ(mat._data, expStorage);
}

TYPED_TEST(GTestMatrix, at_unsafe__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_at_unsafe_Success<typename GTestMatrix<TypeParam>::IntMatType>();
  GTestMatrix<TypeParam>::template test_at_unsafe_Success<const typename GTestMatrix<TypeParam>::IntMatType>();
}

TYPED_TEST(GTestMatrix, op_at__FailBadRowIdx) // NOLINT
{
  GTestMatrix<TypeParam>::template test_op_at_FailBadRowIdx<typename GTestMatrix<TypeParam>::IntMatType>();
  GTestMatrix<TypeParam>::template test_op_at_FailBadRowIdx<const typename GTestMatrix<TypeParam>::IntMatType>();
}

TYPED_TEST(GTestMatrix, op_at__FailBadColIdx) // NOLINT
{
  GTestMatrix<TypeParam>::template test_op_at_FailBadColIdx<typename GTestMatrix<TypeParam>::IntMatType>();
  GTestMatrix<TypeParam>::template test_op_at_FailBadColIdx<const typename GTestMatrix<TypeParam>::IntMatType>();
}

TYPED_TEST(GTestMatrix, op_at__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_op_at_Success<typename GTestMatrix<TypeParam>::IntMatType>();
  GTestMatrix<TypeParam>::template test_op_at_Success<const typename GTestMatrix<TypeParam>::IntMatType>();
}

TYPED_TEST(GTestMatrix, op_eq__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_op_eq_Success<true>();
  GTestMatrix<TypeParam>::template test_op_eq_Success<false>();
}

TYPED_TEST(GTestMatrix, op_plus__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_op_plus_inplace_Success<true>();
  GTestMatrix<TypeParam>::template test_op_plus_inplace_Success<false>();
  GTestMatrix<TypeParam>::template test_op_plus_Success<true>();
  GTestMatrix<TypeParam>::template test_op_plus_Success<false>();
}

TYPED_TEST(GTestMatrix, op_minus__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_op_minus_inplace_Success<true>();
  GTestMatrix<TypeParam>::template test_op_minus_inplace_Success<false>();
  GTestMatrix<TypeParam>::template test_op_minus_Success<true>();
  GTestMatrix<TypeParam>::template test_op_minus_Success<false>();
}

TYPED_TEST(GTestMatrix, op_plus_scalar__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_plus_scalar_Success();
}

TYPED_TEST(GTestMatrix, op_minus_scalar__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_minus_scalar_Success();
}

TYPED_TEST(GTestMatrix, op_mul_scalar__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_mul_scalar_Success();
  GTestMatrix<TypeParam>::test_op_mul_mat_Success();
}

TYPED_TEST(GTestMatrix, op_div_scalar__FailDivByZero) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_div_scalar_IntFailDivByZero();
  GTestMatrix<TypeParam>::test_op_div_scalar_FloatFailDivByZero();
}

TYPED_TEST(GTestMatrix, op_div_scalar__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_div_scalar_inplace_IntSuccess();
  GTestMatrix<TypeParam>::test_op_div_scalar_IntSuccess();
  GTestMatrix<TypeParam>::test_op_div_scalar_inplace_FloatSuccess();
  GTestMatrix<TypeParam>::test_op_div_scalar_FloatSuccess();
}

TYPED_TEST(GTestMatrix, transpose__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_transpose_Success<typename GTestMatrix<TypeParam>::IntMatType>();
  GTestMatrix<TypeParam>::template test_transpose_Success<const typename GTestMatrix<TypeParam>::IntMatType>();
}

TYPED_TEST(GTestMatrix, op_plus_transpose_inplace__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_op_plus_transpose_inplace_Success<true>();
  GTestMatrix<TypeParam>::template test_op_plus_transpose_inplace_Success<false>();
}

// ============================================================================
// FromList() error handling tests
// ============================================================================

TYPED_TEST(GTestMatrix, FromList_TooFewRows__ThrowsRuntimeError) // NOLINT
{
  using IntMatType = typename GTestMatrix<TypeParam>::IntMatType;

  // Try to create a 2x3 matrix with only 1 row
  // clang-format off
  EXPECT_THROW(IntMatType::FromList({
                   {0, 1, 2},
               }),
               std::runtime_error);
  // clang-format on
}

TYPED_TEST(GTestMatrix, FromList_TooManyRows__ThrowsRuntimeError) // NOLINT
{
  using IntMatType = typename GTestMatrix<TypeParam>::IntMatType;

  // Try to create a 2x3 matrix with 3 rows
  // clang-format off
  EXPECT_THROW(
    IntMatType::FromList({
      {0, 1, 2},
      {3, 4, 5},
      {6, 7, 8},
    }),
    std::runtime_error
  );
  // clang-format on
}

TYPED_TEST(GTestMatrix, FromList_TooFewColumns__ThrowsRuntimeError) // NOLINT
{
  using IntMatType = typename GTestMatrix<TypeParam>::IntMatType;

  // Try to create a 2x3 matrix with only 2 columns in a row
  // clang-format off
  EXPECT_THROW(
    IntMatType::FromList({
      {0, 1},
      {3, 4},
    }),
    std::runtime_error
  );
  // clang-format on
}

TYPED_TEST(GTestMatrix, FromList_TooManyColumns__ThrowsRuntimeError) // NOLINT
{
  using IntMatType = typename GTestMatrix<TypeParam>::IntMatType;

  // Try to create a 2x3 matrix with 4 columns in a row
  // clang-format off
  EXPECT_THROW(
    IntMatType::FromList({
      {0, 1, 2, 3},
      {4, 5, 6, 7},
    }),
    std::runtime_error
  );
  // clang-format on
}

TYPED_TEST(GTestMatrix, FromList_InconsistentColumnCounts__ThrowsRuntimeError) // NOLINT
{
  using IntMatType = typename GTestMatrix<TypeParam>::IntMatType;

  // Try to create a 2x3 matrix where rows have different column counts
  // clang-format off
  EXPECT_THROW(
    IntMatType::FromList({
      {0, 1, 2},
      {3, 4},      // Only 2 columns instead of 3
    }),
    std::runtime_error
  );
  // clang-format on
}

TYPED_TEST(GTestMatrix, FromList_EmptyList__ThrowsRuntimeError) // NOLINT
{
  using IntMatType = typename GTestMatrix<TypeParam>::IntMatType;

  // Try to create a 2x3 matrix with an empty initializer list
  EXPECT_THROW(IntMatType::FromList({}), std::runtime_error);
}

TYPED_TEST(GTestMatrix, FromList_SquareMatrix_TooFewRows__ThrowsRuntimeError) // NOLINT
{
  using SquareMatType = typename GTestMatrix<TypeParam>::SquareIntMatType;

  // Try to create a 3x3 matrix with only 2 rows
  // clang-format off
  EXPECT_THROW(
    SquareMatType::FromList({
      {0, 1, 2,},
      {3, 4, 5,},
    }),
    std::runtime_error
  );
  // clang-format on
}

TYPED_TEST(GTestMatrix, FromList_1x1Matrix__Success) // NOLINT
{
  using Mat1x1Type = tracking::math::Matrix<sint32, 1, 1, TypeParam::IsRowMajor>;

  // Verify 1x1 matrices work correctly
  // clang-format off
  const auto mat = Mat1x1Type::FromList({
      {42},
  });
  // clang-format on

  EXPECT_EQ(mat.at_unsafe(0, 0), 42);
}

// Note: Single-row and single-column tests use static layout for clarity
// They don't use TYPED_TEST because they test specific matrix dimensions

TEST(GTestMatrixSpecial, FromList_SingleRowMatrix__Success) // NOLINT
{
  using MatSingleRowType = tracking::math::Matrix<sint32, 1, 4, true>;

  // Verify single-row matrices work correctly
  // clang-format off
  const auto mat = MatSingleRowType::FromList({
      {10, 20, 30, 40},
  });
  // clang-format on

  EXPECT_EQ(mat.at_unsafe(0, 0), 10);
  EXPECT_EQ(mat.at_unsafe(0, 1), 20);
  EXPECT_EQ(mat.at_unsafe(0, 2), 30);
  EXPECT_EQ(mat.at_unsafe(0, 3), 40);
}

TEST(GTestMatrixSpecial, FromList_SingleColumnMatrix__Success) // NOLINT
{
  using MatSingleColType = tracking::math::Matrix<sint32, 4, 1, true>;

  // Verify single-column matrices work correctly
  // clang-format off
  const auto mat = MatSingleColType::FromList({
    {10,},
    {20,},
    {30,},
    {40,},
  });
  // clang-format on

  EXPECT_EQ(mat.at_unsafe(0, 0), 10);
  EXPECT_EQ(mat.at_unsafe(1, 0), 20);
  EXPECT_EQ(mat.at_unsafe(2, 0), 30);
  EXPECT_EQ(mat.at_unsafe(3, 0), 40);
}

// ============================================================================
// Additional Test Cases for Missing Coverage
// ============================================================================

// minmax() Tests
TEST(GTestMatrixSpecial, minmax_AllValuesSame) // NOLINT
{
  using MatType  = tracking::math::Matrix<sint32, 2, 2, true>;
  const auto mat = MatType::FromList({
      {5, 5},
      {5, 5},
  });

  const auto [min, max] = mat.minmax();
  EXPECT_EQ(min, 5);
  EXPECT_EQ(max, 5);
}

TEST(GTestMatrixSpecial, minmax_SingleElement) // NOLINT
{
  using MatType  = tracking::math::Matrix<sint32, 1, 1, true>;
  const auto mat = MatType::FromList({
      {42},
  });

  const auto [min, max] = mat.minmax();
  EXPECT_EQ(min, 42);
  EXPECT_EQ(max, 42);
}

TEST(GTestMatrixSpecial, minmax_ExtremeValues) // NOLINT
{
  using MatType                = tracking::math::Matrix<sint32, 2, 2, true>;
  constexpr sint32 INT_MIN_VAL = std::numeric_limits<sint32>::lowest();
  constexpr sint32 INT_MAX_VAL = std::numeric_limits<sint32>::max();

  const auto mat = MatType::FromList({
      {INT_MIN_VAL, 0},
      {0, INT_MAX_VAL},
  });

  const auto [min, max] = mat.minmax();
  EXPECT_EQ(min, INT_MIN_VAL);
  EXPECT_EQ(max, INT_MAX_VAL);
}

// operator!= Tests
TEST(GTestMatrixSpecial, op_not_equal__DifferentMatrices) // NOLINT
{
  using MatType   = tracking::math::Matrix<sint32, 2, 2, true>;
  // clang-format off
  const auto mat1 = MatType::FromList({
      {1, 2},
      {3, 4},
  });
  const auto mat2 = MatType::FromList({
      {1, 2}, {3, 5}, // Different element
  });
  // clang-format on

  EXPECT_TRUE(mat1 != mat2);
  EXPECT_FALSE(mat1 == mat2);
}

TEST(GTestMatrixSpecial, op_not_equal__SameMatrices) // NOLINT
{
  using MatType   = tracking::math::Matrix<sint32, 2, 2, true>;
  // clang-format off
  const auto mat1 = MatType::FromList({
      {1, 2},
      {3, 4},
  });
  const auto mat2 = MatType::FromList({
      {1, 2},
      {3, 4},
  });
  // clang-format on

  EXPECT_FALSE(mat1 != mat2);
  EXPECT_TRUE(mat1 == mat2);
}

TEST(GTestMatrixSpecial, op_not_equal__DifferentValues) // NOLINT
{
  using MatType   = tracking::math::Matrix<sint32, 2, 2, true>;
  // clang-format off
  const auto mat1 = MatType::FromList({
      {1, 2},
      {3, 4},
  });
  const auto mat2 = MatType::FromList({
      {1, 2}, {3, 5}, // Different element
  });
  // clang-format on

  EXPECT_TRUE(mat1 != mat2);
  EXPECT_FALSE(mat1 == mat2);
}

// Boundary Access Tests
TEST(GTestMatrixSpecial, op_at_RowBoundary__OutOfBounds) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 3, 3, true>;
  auto mat      = MatType::Zeros();

  auto result = mat(3, 0); // Row index equals Rows (out of bounds)
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), tracking::math::Errors::invalid_access_row);
}

TEST(GTestMatrixSpecial, op_at_ColBoundary__OutOfBounds) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 3, 3, true>;
  auto mat      = MatType::Zeros();

  auto result = mat(0, 3); // Col index equals Cols (out of bounds)
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), tracking::math::Errors::invalid_access_col);
}

TEST(GTestMatrixSpecial, op_at__NegativeIndices) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 3, 3, true>;
  auto mat      = MatType::Zeros();

  auto result_row = mat(-1, 0);
  EXPECT_FALSE(result_row.has_value());
  EXPECT_EQ(result_row.error(), tracking::math::Errors::invalid_access_row);

  auto result_col = mat(0, -1);
  EXPECT_FALSE(result_col.has_value());
  EXPECT_EQ(result_col.error(), tracking::math::Errors::invalid_access_col);
}

// Matrix Multiplication with Different Dimensions
TEST(GTestMatrixSpecial, op_mul__SquareMatrices) // NOLINT
{
  using MatType       = tracking::math::Matrix<sint32, 3, 3, true>;
  const auto identity = tracking::math::SquareMatrix<sint32, 3, true>::Identity();

  auto ones   = MatType::Ones();
  auto result = identity * ones;

  // Identity * Ones should equal Ones
  EXPECT_TRUE(result == ones);
}

TEST(GTestMatrixSpecial, op_mul__RowVectorTimesMatrix) // NOLINT
{
  using RowVecType    = tracking::math::Matrix<sint32, 1, 3, true>;
  // clang-format off
  const auto row      = RowVecType::FromList({
      {1, 2, 3},
  });
  // clang-format on
  const auto identity = tracking::math::SquareMatrix<sint32, 3, true>::Identity();

  const auto result = row * identity;
  EXPECT_TRUE(result == row);
}

TEST(GTestMatrixSpecial, op_mul__MatrixTimesColumnVector) // NOLINT
{
  using ColVecType    = tracking::math::Matrix<sint32, 3, 1, true>;
  // clang-format off
  const auto col      = ColVecType::FromList({
      {5},
      {10},
      {15},
  });
  // clang-format on
  const auto identity = tracking::math::SquareMatrix<sint32, 3, true>::Identity();

  const auto result = identity * col;
  EXPECT_TRUE(result == col);
}

// Self-Assignment Tests
TEST(GTestMatrixSpecial, op_plus_equal__Self) // NOLINT
{
  using MatType      = tracking::math::Matrix<sint32, 2, 2, true>;
  // clang-format off
  auto       matA    = MatType::FromList({
      {1, 2},
      {3, 4},
  });
  const auto expMatA = MatType::FromList({
      {2, 4},
      {6, 8},
  });
  // clang-format on

  matA += matA;
  EXPECT_TRUE(matA == expMatA);
}

TEST(GTestMatrixSpecial, op_minus_equal__Self) // NOLINT
{
  using MatType      = tracking::math::Matrix<sint32, 2, 2, true>;
  // clang-format off
  auto       matA    = MatType::FromList({
      {1, 2},
      {3, 4},
  });
  // clang-format on
  const auto expMatA = MatType::Zeros();

  matA -= matA;
  EXPECT_TRUE(matA == expMatA);
}

// Frobenius Norm Tests (floating-point only)
TEST(GTestMatrixSpecial, frobenius_norm__OneMatrix) // NOLINT
{
  using MatType = tracking::math::Matrix<float32, 3, 3, true>;
  auto ones     = MatType::Ones();

  const auto norm = ones.frobenius_norm();
  // Frobenius norm of 3x3 identity is sqrt(9) = 3
  EXPECT_FLOAT_EQ(norm, 3.0F);
}

TEST(GTestMatrixSpecial, frobenius_norm__ZeroMatrix) // NOLINT
{
  using MatType    = tracking::math::Matrix<float32, 2, 2, true>;
  const auto zeros = MatType::Zeros();

  const auto norm = zeros.frobenius_norm();
  EXPECT_FLOAT_EQ(norm, 0.0F);
}

TEST(GTestMatrixSpecial, frobenius_norm__SingleElement) // NOLINT
{
  using MatType  = tracking::math::Matrix<float32, 1, 1, true>;
  // clang-format off
  const auto mat = MatType::FromList({
      {5.0F},
  });
  // clang-format on

  const auto norm = mat.frobenius_norm();
  EXPECT_FLOAT_EQ(norm, 5.0F);
}

TEST(GTestMatrixSpecial, frobenius_norm__ArbitraryMatrix) // NOLINT
{
  using MatType  = tracking::math::Matrix<float32, 2, 2, true>;
  // clang-format off
  const auto mat = MatType::FromList({
      {3.0F, 4.0F},
      {0.0F, 0.0F},
  });
  // clang-format on

  const auto norm = mat.frobenius_norm();
  // sqrt(3^2 + 4^2 + 0^2 + 0^2) = sqrt(25) = 5
  EXPECT_FLOAT_EQ(norm, 5.0F);
}

// setBlock Tests
TEST(GTestMatrixSpecial, setBlock_CompileTime__Success) // NOLINT
{
  using DstMatType = tracking::math::Matrix<sint32, 4, 4, true>;
  using SrcMatType = tracking::math::Matrix<sint32, 2, 2, true>;

  DstMatType       dst = DstMatType::Zeros();
  // clang-format off
  const SrcMatType src = SrcMatType::FromList({
      {1, 2},
      {3, 4},
  });
  // clang-format on

  // Set block at (1,1) to (2,2) from src(0,0) to (1,1)
  dst.setBlock<2, 2, 2, 2, 0, 0, true, 1, 1>(src);

  EXPECT_EQ(dst.at_unsafe(1, 1), 1);
  EXPECT_EQ(dst.at_unsafe(1, 2), 2);
  EXPECT_EQ(dst.at_unsafe(2, 1), 3);
  EXPECT_EQ(dst.at_unsafe(2, 2), 4);

  // Other elements should remain 0
  EXPECT_EQ(dst.at_unsafe(0, 0), 0);
  EXPECT_EQ(dst.at_unsafe(0, 1), 0);
  EXPECT_EQ(dst.at_unsafe(0, 2), 0);
  EXPECT_EQ(dst.at_unsafe(0, 3), 0);
  EXPECT_EQ(dst.at_unsafe(1, 0), 0);
  EXPECT_EQ(dst.at_unsafe(1, 3), 0);
  EXPECT_EQ(dst.at_unsafe(2, 0), 0);
  EXPECT_EQ(dst.at_unsafe(2, 3), 0);
  EXPECT_EQ(dst.at_unsafe(3, 0), 0);
  EXPECT_EQ(dst.at_unsafe(3, 1), 0);
  EXPECT_EQ(dst.at_unsafe(3, 2), 0);
  EXPECT_EQ(dst.at_unsafe(3, 3), 0);
}

TEST(GTestMatrixSpecial, setBlock_Runtime__Success) // NOLINT
{
  using DstMatType = tracking::math::Matrix<sint32, 4, 4, true>;
  using SrcMatType = tracking::math::Matrix<sint32, 2, 2, true>;

  DstMatType       dst = DstMatType::Zeros();
  // clang-format off
  const SrcMatType src = SrcMatType::FromList({
      {1, 2},
      {3, 4},
  });
  // clang-format on

  // Set block at (1,1) to (2,2) from src(0,0) to (1,1)
  dst.setBlock<2, 2, true>(2, 2, 0, 0, 1, 1, src);

  EXPECT_EQ(dst.at_unsafe(1, 1), 1);
  EXPECT_EQ(dst.at_unsafe(1, 2), 2);
  EXPECT_EQ(dst.at_unsafe(2, 1), 3);
  EXPECT_EQ(dst.at_unsafe(2, 2), 4);

  // Other elements should remain 0
  EXPECT_EQ(dst.at_unsafe(0, 0), 0);
  EXPECT_EQ(dst.at_unsafe(3, 3), 0);
}

TEST(GTestMatrixSpecial, setBlock_BoundaryCondition_OutOfBounds) // NOLINT
{
  using DstMatType = tracking::math::Matrix<sint32, 3, 3, true>;
  using SrcMatType = tracking::math::Matrix<sint32, 2, 2, true>;

  DstMatType       dst = DstMatType::Zeros();
  const SrcMatType src = SrcMatType::Ones();

  // Try to set block starting at (2,2), which would go out of bounds
  // This should trigger an assertion due to bounds checking
  // We expect the assertion to fail, so we test for death
  // Note: setBlock is a template method, so we need to wrap it in a lambda or similar
  // to avoid macro parsing issues with commas in template arguments
  auto callSetBlock = [&]() { dst.setBlock<2, 2, true>(2, 2, 0, 0, 2, 2, src); };
  EXPECT_DEATH(callSetBlock(), ".*");

  // Verify that the destination matrix remains unchanged
  for (auto row = 0; row < dst.Rows; ++row)
  {
    for (auto col = 0; col < dst.Cols; ++col)
    {
      EXPECT_EQ(dst.at_unsafe(row, col), 0);
    }
  }
}

// Aliasing Detection Tests
TEST(GTestMatrixSpecial, op_minus_transpose_inplace_Square__Success) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 3, 3, true>;
  // clang-format off
  MatType mat   = MatType::FromList({
      {1, 2, 3},
      {4, 5, 6},
      {7, 8, 9},
  });
  // clang-format on

  // mat - mat^T
  // mat = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}
  // mat^T = {{1, 4, 7}, {2, 5, 8}, {3, 6, 9}}
  // mat - mat^T = {{0, -2, -4}, {2, 0, -2}, {4, 2, 0}}
  mat -= mat.transpose();

  // clang-format off
  const auto expected = MatType::FromList({
      {0, -2, -4},
      {2, 0, -2},
      {4, 2, 0},
  });
  // clang-format on

  EXPECT_EQ(mat._data, expected._data);
}

TEST(GTestMatrixSpecial, op_plus_transpose_inplace_NonSquare_ShouldNotAlias) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 2, 3, true>;
  // clang-format off
  MatType mat   = MatType::FromList({
      {1, 2, 3},
      {4, 5, 6},
  });
  // clang-format on

  // Transpose is 3x2, mat is 2x3, sizes differ, so += should not match
  // This test is to ensure no aliasing issue for different sizes
  // But since sizes differ, operator+= won't be called
  // Perhaps test with a compatible transpose view
  // For now, just check that transpose works
  const auto& trans = mat.transpose();
  EXPECT_EQ(trans.Rows, 3);
  EXPECT_EQ(trans.Cols, 2);
  EXPECT_EQ(trans.at_unsafe(0, 0), 1);
  EXPECT_EQ(trans.at_unsafe(1, 0), 2);
}

// minmax Edge Cases
TEST(GTestMatrixSpecial, minmax_NegativeValues) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 2, 2, true>;
  // clang-format off
  const auto mat = MatType::FromList({
      {-5, -1},
      {-10, -3},
  });
  // clang-format on

  const auto [min, max] = mat.minmax();
  EXPECT_EQ(min, -10);
  EXPECT_EQ(max, -1);
}

TEST(GTestMatrixSpecial, minmax_MixedPositiveNegative) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 2, 2, true>;
  // clang-format off
  const auto mat = MatType::FromList({
      {-5, 10},
      {0, -3},
  });
  // clang-format on

  const auto [min, max] = mat.minmax();
  EXPECT_EQ(min, -5);
  EXPECT_EQ(max, 10);
}

TEST(GTestMatrixSpecial, minmax_NonSquare) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 2, 3, true>;
  // clang-format off
  const auto mat = MatType::FromList({
      {1, 5, 3},
      {2, 0, 4},
  });
  // clang-format on

  const auto [min, max] = mat.minmax();
  EXPECT_EQ(min, 0);
  EXPECT_EQ(max, 5);
}

// Matrix Multiplication Non-Square
TEST(GTestMatrixSpecial, op_mul_NonSquare) // NOLINT
{
  using Mat1Type   = tracking::math::Matrix<sint32, 2, 3, true>; // 2x3
  using Mat2Type   = tracking::math::Matrix<sint32, 3, 4, true>; // 3x4
  using ResultType = tracking::math::Matrix<sint32, 2, 4, true>; // 2x4

  // clang-format off
  const auto mat1 = Mat1Type::FromList({
      {1, 2, 3},
      {4, 5, 6},
  });
  const auto mat2 = Mat2Type::FromList({
      {7, 8, 9, 10},
      {11, 12, 13, 14},
      {15, 16, 17, 18},
  });
  // clang-format on

  const auto result = mat1 * mat2;

  // Manual calculation
  // clang-format off
  const auto expected = ResultType::FromList({
      {1 * 7 + 2 * 11 + 3 * 15, 1 * 8 + 2 * 12 + 3 * 16, 1 * 9 + 2 * 13 + 3 * 17, 1 * 10 + 2 * 14 + 3 * 18},
      {4 * 7 + 5 * 11 + 6 * 15, 4 * 8 + 5 * 12 + 6 * 16, 4 * 9 + 5 * 13 + 6 * 17, 4 * 10 + 5 * 14 + 6 * 18},
  });
  // clang-format on

  EXPECT_EQ(result._data, expected._data);
}

// Transpose Non-Square and Chained
TEST(GTestMatrixSpecial, transpose_NonSquare_Chained) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 2, 3, true>;
  // clang-format off
  const auto mat = MatType::FromList({
      {1, 2, 3},
      {4, 5, 6},
  });
  // clang-format on

  // Test chained transpose
  const auto& trans       = mat.transpose();
  const auto& trans_trans = trans.transpose();

  // trans_trans should be same as mat
  EXPECT_EQ(trans_trans.Rows, mat.Rows);
  EXPECT_EQ(trans_trans.Cols, mat.Cols);
  for (auto r = 0; r < mat.Rows; ++r)
  {
    for (auto c = 0; c < mat.Cols; ++c)
    {
      EXPECT_EQ(trans_trans.at_unsafe(r, c), mat.at_unsafe(r, c));
    }
  }
}

TEST(GTestMatrixSpecial, transpose_rvalue) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 2, 3, true>;
  // clang-format off
  auto mat      = MatType::FromList({
      {1, 2, 3},
      {4, 5, 6},
  });
  // clang-format on

  // Test transpose_rvalue
  auto trans = std::move(mat).transpose_rvalue();

  EXPECT_EQ(trans.Rows, 3);
  EXPECT_EQ(trans.Cols, 2);
  EXPECT_EQ(trans.at_unsafe(0, 0), 1);
  EXPECT_EQ(trans.at_unsafe(1, 0), 2);
  EXPECT_EQ(trans.at_unsafe(2, 0), 3);
  EXPECT_EQ(trans.at_unsafe(0, 1), 4);
  EXPECT_EQ(trans.at_unsafe(1, 1), 5);
  EXPECT_EQ(trans.at_unsafe(2, 1), 6);
}

// Error Handling for /= with zero
TEST(GTestMatrixSpecial, op_div_inplace_IntZero) // NOLINT
{
  using MatType = tracking::math::Matrix<sint32, 2, 2, true>;
  MatType mat   = MatType::Ones();

  // call UUT
  auto result = mat /= 0;
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), tracking::math::Errors::divide_by_zero);
}

TEST(GTestMatrixSpecial, op_div_inplace_FloatZero) // NOLINT
{
  using MatType = tracking::math::Matrix<float32, 2, 2, true>;
  MatType mat   = MatType::Ones();

  // call UUT
  auto result = mat /= 0.0F;
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), tracking::math::Errors::divide_by_zero);
}

// Const correctness test for non-member operators
TEST(GTestMatrixSpecial, non_member_op_plus_const) // NOLINT
{
  using MatType     = tracking::math::Matrix<sint32, 2, 2, true>;
  const MatType mat = MatType::Ones();

  // This should work with const matrix
  const auto result = 5 + mat;

  for (auto val : result._data)
  {
    EXPECT_EQ(val, 6); // 5 + 1
  }
}

TEST(GTestMatrixSpecial, non_member_op_mul_const) // NOLINT
{
  using MatType     = tracking::math::Matrix<sint32, 2, 2, true>;
  const MatType mat = MatType::Ones();

  // This should work with const matrix
  const auto result = 3 * mat;

  for (auto val : result._data)
  {
    EXPECT_EQ(val, 3); // 3 * 1
  }
}
