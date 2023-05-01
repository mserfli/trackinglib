#include "gtest/gtest.h"
#include "trackingLib/math/linalg/matrix.hpp"
#include <type_traits>

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
  IntMatType                   _testIntMat{};
  IntMatTypeTranspMemLayout    _testIntMatTransposed{};
  IntMatMulType                _testIntMatMul{};
  IntMatMulTypeTranspMemLayout _testIntMatMulTransposed{};
  IntMatMulResultType          _testIntMatMulResult{};
  FloatMatType                 _testFloatMat{};

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
    EXPECT_EQ(retVal.error(), T::Errors::INVALID_ACCESS_ROW);
  }

  template <typename T>
  void test_op_at_FailBadColIdx()
  {
    T mat{_testIntMat};
    // call UUT
    auto retVal = mat(0, T::Cols);

    EXPECT_FALSE(retVal.has_value());
    EXPECT_EQ(retVal.error(), T::Errors::INVALID_ACCESS_COL);
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

  template <>
  void test_op_eq_Success<true>()
  {
    IntMatType       mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    auto isEqual = (mat == other);

    EXPECT_TRUE(isEqual);
  }

  template <>
  void test_op_eq_Success<false>()
  {
    const IntMatType                mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    auto isEqual = (mat == other);

    EXPECT_TRUE(isEqual);
  }

  void test_op_plus_inplace_Success()
  {
    IntMatType       mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    mat += other;

    for (auto idx = 0; idx < _testIntMat._data.size(); ++idx)
    {
      EXPECT_EQ(mat._data[idx], 2 * _testIntMat._data[idx]);
    }
  }

  template <bool SameMajorOrder>
  void test_op_plus_Success();

  template <>
  void test_op_plus_Success<true>()
  {
    const IntMatType mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    auto res = mat + other;

    for (auto idx = 0; idx < res._data.size(); ++idx)
    {
      EXPECT_EQ(res._data[idx], 2 * _testIntMat._data[idx]);
    }
  }

  template <>
  void test_op_plus_Success<false>()
  {
    const IntMatType                mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    auto res = mat + other;

    for (auto idx = 0; idx < res._data.size(); ++idx)
    {
      EXPECT_EQ(res._data[idx], 2 * _testIntMat._data[idx]);
    }
  }

  void test_op_minus_inplace_Success()
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

  template <bool SameMajorOrder>
  void test_op_minus_Success();

  template <>
  void test_op_minus_Success<true>()
  {
    const IntMatType mat{_testIntMat};
    const IntMatType other{_testIntMat};

    // call UUT
    auto res = mat - other;

    for (auto idx = 0; idx < res._data.size(); ++idx)
    {
      EXPECT_EQ(res._data[idx], 0);
    }
  }

  template <>
  void test_op_minus_Success<false>()
  {
    const IntMatType                mat{_testIntMat};
    const IntMatTypeTranspMemLayout other{_testIntMatTransposed};

    // call UUT
    auto res = mat - other;

    for (auto idx = 0; idx < res._data.size(); ++idx)
    {
      EXPECT_EQ(res._data[idx], 0);
    }
  }

  void test_op_mul_scalar_Success()
  {
    IntMatType mat{_testIntMat};
    IntMatType res{};

    // call UUT
    res = 2 * mat;

    for (auto idx = 0; idx < _testIntMat._data.size(); ++idx)
    {
      EXPECT_EQ(res._data[idx], 2 * _testIntMat._data[idx]);
    }
  }

  void test_op_div_scalar_inplace_IntSuccess()
  {
    IntMatType mat{_testIntMat};

    // call UUT
    mat /= 2;

    for (auto idx = 0; idx < _testIntMat._data.size(); ++idx)
    {
      EXPECT_EQ(mat._data[idx], _testIntMat._data[idx] / 2);
    }
  }

  void test_op_div_scalar_inplace_FloatSuccess()
  {
    FloatMatType mat{_testFloatMat};

    // call UUT
    mat /= 2.0F;

    for (auto idx = 0; idx < _testFloatMat._data.size(); ++idx)
    {
      EXPECT_EQ(mat._data[idx], _testFloatMat._data[idx] / 2.0F);
    }
  }

  void test_op_div_scalar_IntFailDivByZero()
  {
    IntMatType mat{_testIntMat};

    // call UUT
    auto res = mat / 0;
    EXPECT_EQ(res.error(), IntMatType::Errors::DIV_BY_ZERO);
  }

  void test_op_div_scalar_FloatFailDivByZero()
  {
    FloatMatType mat{_testFloatMat};

    // call UUT
    auto res = mat / 0.0F;
    EXPECT_EQ(res.error(), FloatMatType::Errors::DIV_BY_ZERO);
  }

  void test_op_mul_mat_Success() 
  { 
    IntMatType mat1{_testIntMat}; 
    IntMatMulType mat2{_testIntMatMul};

    // call UUT
    auto res = mat1 * mat2;

    EXPECT_TRUE(res==_testIntMatMulResult);
  }

  template <typename T>
  void test_transpose_Success()
  {
    T mat{_testIntMat};
    // clang-format off
    typename T::transpose_type_row_major expMat({
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

/// \brief template specialization of SetUp initializing the members for RowMajor memory layout
template <>
void GTestMatrix<MatrixStorageType<true>>::SetUp()
{
  // clang-format off
  _testIntMat = IntMatType({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  _testIntMatTransposed = IntMatTypeTranspMemLayout({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  _testIntMatMul = IntMatMulType({
    {0,  1,  2,  3,},
    {4,  5,  6,  7,},
    {8,  9, 10, 11,},
  });
  _testIntMatMulTransposed = IntMatMulTypeTranspMemLayout({
    {0,  4,  8,},
    {1,  5,  9,},
    {2,  6, 10,},
    {3,  7, 11,},
  });
  _testIntMatMulResult = IntMatMulResultType({
    {20, 23, 26, 29,},
    {56, 68, 80, 92,},
  });
  _testFloatMat = FloatMatType({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  // clang-format on
}

/// \brief template specialization of CTOR test with initializer list for RowMajor memory layout
template <>
void GTestMatrix<MatrixStorageType<true>>::test_ctor_initializerList_Success()
{
  // clang-format off
  // call UUT
  const IntMatType mat({
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
  _testIntMat = IntMatType({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  _testIntMatTransposed = IntMatTypeTranspMemLayout({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  _testIntMatMul = IntMatMulType({
    {0,  4,  8,},
    {1,  5,  9,},
    {2,  6, 10,},
    {3,  7, 11,},
  });
  _testIntMatMulTransposed = IntMatMulTypeTranspMemLayout({
    {0,  1,  2,  3,},
    {4,  5,  6,  7,},
    {8,  9, 10, 11,},
  });
  _testIntMatMulResult = IntMatMulResultType({
    {20, 56,},
    {23, 68,},
    {26, 80,},
    {29, 92,},  
  });
  _testFloatMat = FloatMatType({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  // clang-format on
}

/// \brief template specialization of CTOR test with initializer list for ColumnMajor memory layout
template <>
void GTestMatrix<MatrixStorageType<false>>::test_ctor_initializerList_Success()
{
  // clang-format off
  // call UUT
  const IntMatType mat({
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
  GTestMatrix<TypeParam>::test_op_plus_inplace_Success();
  GTestMatrix<TypeParam>::template test_op_plus_Success<true>();
  GTestMatrix<TypeParam>::template test_op_plus_Success<false>();
}

TYPED_TEST(GTestMatrix, op_minus__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_minus_inplace_Success();
  GTestMatrix<TypeParam>::template test_op_minus_Success<true>();
  GTestMatrix<TypeParam>::template test_op_minus_Success<false>();
}

TYPED_TEST(GTestMatrix, op_mul__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_mul_scalar_Success();
  GTestMatrix<TypeParam>::test_op_mul_mat_Success();
}

TYPED_TEST(GTestMatrix, op_div__FailDivByZero) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_div_scalar_IntFailDivByZero();
  GTestMatrix<TypeParam>::test_op_div_scalar_FloatFailDivByZero();
}

TYPED_TEST(GTestMatrix, op_div__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_div_scalar_inplace_IntSuccess();
  GTestMatrix<TypeParam>::test_op_div_scalar_inplace_FloatSuccess();
}

TYPED_TEST(GTestMatrix, transpose__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_transpose_Success<typename GTestMatrix<TypeParam>::IntMatType>();
  GTestMatrix<TypeParam>::template test_transpose_Success<const typename GTestMatrix<TypeParam>::IntMatType>();
}
