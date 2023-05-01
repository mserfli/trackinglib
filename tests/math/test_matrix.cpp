#include "gtest/gtest.h"
#include "trackingLib/math/linalg/matrix.hpp"
#include <type_traits>

#define TEST_MATRIX_WITH_VISUAL_INSPECTION(X) // X.print()

// helper class to support typed tests which wraps the IsRowMajor param into a type
template <bool IsRowMajor_>
struct MatrixStorageType
{
  static constexpr auto IsRowMajor = IsRowMajor_;
};

template <typename MatrixStorageType>
class GTestMatrix: public ::testing::Test
{
public:
  void SetUp() final;

protected:
  using IntMatType   = tracking::math::Matrix<sint32, 2, 3, MatrixStorageType::IsRowMajor>;
  using FloatMatType = tracking::math::Matrix<float32, 2, 3, MatrixStorageType::IsRowMajor>;
  IntMatType   _testIntMat{};
  FloatMatType _testFloatMat{};

  void test_ctor_initializerList_Success();

  template <typename T>
  void test_at_unsafe_Success()
  {
    T                      mat{_testIntMat};
    typename T::value_type retVal = -1;
    // call UUT
    retVal = mat.at_unsafe(T::RowsOut - 1, T::ColsOut - 1);

    EXPECT_EQ(retVal, 5);
  }

  template <typename T>
  void test_op_at_FailBadRowIdx()
  {
    T mat{_testIntMat};
    // call UUT
    auto retVal = mat(T::RowsOut, 0);

    EXPECT_FALSE(retVal.has_value());
    EXPECT_EQ(retVal.error(), T::Errors::INVALID_ACCESS_ROW);
  }

  template <typename T>
  void test_op_at_FailBadColIdx()
  {
    T mat{_testIntMat};
    // call UUT
    auto retVal = mat(0, T::ColsOut);

    EXPECT_FALSE(retVal.has_value());
    EXPECT_EQ(retVal.error(), T::Errors::INVALID_ACCESS_COL);
  }

  template <typename T>
  void test_op_at_Success()
  {
    T mat{_testIntMat};
    // call UUT
    auto retVal = mat(T::RowsOut - 1, T::ColsOut - 1);

    EXPECT_TRUE(retVal.has_value());
    EXPECT_EQ(retVal, 5);
  }

  void test_op_plus_unary_Success()
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

  void test_op_minus_unary_Success()
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

  void test_op_mul_scalar_unary_Success()
  {
    IntMatType mat{_testIntMat};

    // call UUT
    mat *= 2;

    for (auto idx = 0; idx < _testIntMat._data.size(); ++idx)
    {
      EXPECT_EQ(mat._data[idx], 2 * _testIntMat._data[idx]);
    }
  }

  void test_op_div_scalar_unary_IntSuccess()
  {
    IntMatType mat{_testIntMat};

    // call UUT
    mat /= 2;

    for (auto idx = 0; idx < _testIntMat._data.size(); ++idx)
    {
      EXPECT_EQ(mat._data[idx], _testIntMat._data[idx] / 2);
    }
  }

  void test_op_div_scalar_unary_FloatSuccess()
  {
    FloatMatType mat{_testFloatMat};

    // call UUT
    mat /= 2.0F;

    for (auto idx = 0; idx < _testFloatMat._data.size(); ++idx)
    {
      EXPECT_EQ(mat._data[idx], _testFloatMat._data[idx] / 2.0F);
    }
  }

  void test_op_div_scalar_unary_IntFailDivByZero()
  {
    IntMatType mat{_testIntMat};

    // call UUT
#ifdef NDEBUG
    auto res = mat.operator/=(0);
    EXPECT_EQ(res.error(), IntMatType::Errors::DIV_BY_ZERO);
#else
    EXPECT_DEATH({ auto res = mat.operator/=(0); }, "");
#endif
  }

  void test_op_div_scalar_unary_FloatFailDivByZero()
  {
    FloatMatType mat{_testFloatMat};

    // call UUT
#ifdef NDEBUG
    auto res = mat.operator/=(0.0F);
    EXPECT_EQ(res.error(), FloatMatType::Errors::DIV_BY_ZERO);
#else
    EXPECT_DEATH({ auto res = mat.operator/=(0.0F); }, "");
#endif
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

    EXPECT_TRUE(matT.RowsOut == expMat.RowsOut);
    EXPECT_TRUE(matT.ColsOut == expMat.ColsOut);
    for (auto row = 0; row < expMat.RowsOut; ++row)
    {
      for (auto col = 0; col < expMat.ColsOut; ++col)
      {
        EXPECT_EQ(matT.at_unsafe(row, col), expMat.at_unsafe(row, col));
      }
    }
  }
};

template <>
void GTestMatrix<MatrixStorageType<true>>::SetUp()
{
  // clang-format off
  _testIntMat = IntMatType({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  _testFloatMat = FloatMatType({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  // clang-format on
}

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

template <>
void GTestMatrix<MatrixStorageType<false>>::SetUp()
{
  // clang-format off
  _testIntMat = IntMatType({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  _testFloatMat = FloatMatType({
    {0, 3,},
    {1, 4,},
    {2, 5,},
  });
  // clang-format on
}

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

TYPED_TEST(GTestMatrix, op_plus__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_plus_unary_Success();
}

TYPED_TEST(GTestMatrix, op_minus__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_minus_unary_Success();
}

TYPED_TEST(GTestMatrix, op_mul__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_mul_scalar_unary_Success();
}

TYPED_TEST(GTestMatrix, op_div__FailDivByZero) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_div_scalar_unary_IntFailDivByZero();
  GTestMatrix<TypeParam>::test_op_div_scalar_unary_FloatFailDivByZero();
}

TYPED_TEST(GTestMatrix, op_div__Success) // NOLINT
{
  GTestMatrix<TypeParam>::test_op_div_scalar_unary_IntSuccess();
  GTestMatrix<TypeParam>::test_op_div_scalar_unary_FloatSuccess();
}

TYPED_TEST(GTestMatrix, transpose__Success) // NOLINT
{
  GTestMatrix<TypeParam>::template test_transpose_Success<typename GTestMatrix<TypeParam>::IntMatType>();
  GTestMatrix<TypeParam>::template test_transpose_Success<const typename GTestMatrix<TypeParam>::IntMatType>();
}


#if 0


TEST(Matrix, op_equal__Success) // NOLINT
{
  using MatType = tracking::math::Matrix<float32, 2, 3>;
  // clang-format off
  MatType matA({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  MatType matB({
    {0, 1, 2,},
    {3, 4, 5,},
  });
  // clang-format on

  // call UUT
  auto res = (matA == matB);

  EXPECT_TRUE(res);
}


#endif
