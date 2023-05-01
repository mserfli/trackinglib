#ifndef FDEAAACC_9EF1_4C87_94DC_2FA494822664
#define FDEAAACC_9EF1_4C87_94DC_2FA494822664

#include "base/first_include.h"
#include "base/atomic_types.h"
#include "math/linalg/contracts/matrix_intf.h"
#include <array>
#include <initializer_list>
#include <iostream>
#include <type_traits>

namespace tracking
{
namespace math
{

// forward declaration to prevent cyclic includes
template <typename ValueType, sint32 Size, bool isLower>
class TriangularMatrix;

// TODO(matthias): add doxygen
// TODO(matthias): add support for external memory
// TODO(matthias): add template params to support coord transformations (inFrame e.g. EGO_k_1, outFrame, power -> 2 for covs)
template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor = true>
class Matrix: public contract::MatrixIntf<Matrix<ValueType, Rows, Cols, IsRowMajor>, Matrix>
{
public:
  using transpose_type            = Matrix<ValueType, Cols, Rows, !IsRowMajor>;
  using transpose_type_row_major  = Matrix<ValueType, Cols, Rows, true>;
  using value_type                = ValueType;
  static constexpr auto RowsOut   = Rows;
  static constexpr auto ColsOut   = Cols;
  static constexpr auto RowsInMem = IsRowMajor ? Rows : Cols;
  static constexpr auto ColsInMem = IsRowMajor ? Cols : Rows;

  enum class Errors
  {
    INVALID_ACCESS_ROW,
    INVALID_ACCESS_COL,
    DIV_BY_ZERO
  };

  // rule of 5 declarations
  Matrix()                  = default;
  Matrix(const Matrix&)     = default;
  Matrix(Matrix&&) noexcept = default;
  auto operator=(const Matrix&) -> Matrix& = default;
  auto operator=(Matrix&&) noexcept -> Matrix& = default;
  virtual ~Matrix()                            = default;

  /// \brief Construct a new Matrix object given initializer list representing the memory layout of the matrix
  /// \param[in] list  An initializer list describing the memory layout of the matrix
  Matrix(const std::initializer_list<std::initializer_list<ValueType>>& list);

  void        setZeros();
  static auto Zeros() -> Matrix;

  void        setOnes();
  static auto Ones() -> Matrix;

  void print() const;

  auto operator()(sint32 row, sint32 col) const -> tl::expected<ValueType, Errors>;
  auto operator()(sint32 row, sint32 col) -> tl::expected<std::reference_wrapper<ValueType>, Errors>;

  auto operator==(const Matrix& other) const -> bool;

  auto operator+=(const Matrix& other) -> Matrix&;
  auto operator-=(const Matrix& other) -> Matrix&;

  auto operator*=(ValueType scalar) -> Matrix&;

  template <typename IntType = ValueType, typename std::enable_if_t<std::is_integral<IntType>::value, bool> = true>
  auto operator/=(IntType scalar) -> tl::expected<std::reference_wrapper<Matrix>, Errors>;

  template <typename FloatType = ValueType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  auto operator/=(FloatType scalar) -> tl::expected<std::reference_wrapper<Matrix>, Errors>;

  auto transpose() const -> const transpose_type&;
  auto transpose() -> transpose_type&;

#if 0
  /// Compliant to AUTOSAR C++14, A8-4-8: Output parameters shall not be used
  template <sint32 Cols2>
  auto operator*=(const Matrix<ValueType, Cols, Cols2>& other) -> Matrix<ValueType, Rows, Cols2>;


  /// \brief Set a block matrix at given position
  /// \tparam SrcRowSize   Rows of the source block
  /// \tparam SrcColSize   Cols of the source block
  /// \tparam SrcRowCount  Number of rows to copy from source
  /// \tparam SrcColCount  Number of cols to copy from source
  /// \tparam SrcRowBeg    Begin row index in source
  /// \tparam SrcColBeg    Begin col index in source
  /// \tparam DstRowBeg    Begin row index in dest
  /// \tparam DstColBeg    Begin col index in dest
  /// \param[in] block     Source block matrix to copy from
  template <sint32 SrcRowSize,
            sint32 SrcColSize,
            sint32 SrcRowCount,
            sint32 SrcColCount,
            sint32 SrcRowBeg,
            sint32 SrcColBeg,
            sint32 DstRowBeg,
            sint32 DstColBeg>
  void setBlock(const Matrix<ValueType, SrcRowSize, SrcColSize>& block);

  auto transpose() const -> transpose_type;

  void print() const;

  explicit Matrix(const std::array<ValueType, static_cast<size_t>(Rows* Cols)>& arr);
#endif

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on
  using Storage = std::array<ValueType, static_cast<sint32>(Rows* Cols)>;
  auto data() const -> const Storage& { return _data; }
  auto data() -> Storage& { return _data; }

  auto at_unsafe(sint32 row, sint32 col) const -> ValueType;
  auto at_unsafe(sint32 row, sint32 col) -> ValueType&;

  template <typename IntType = ValueType, typename std::enable_if_t<std::is_integral<IntType>::value, bool> = true>
  auto inplace_div_by_int_unsafe(IntType scalar) -> Matrix&;

  template <typename FloatType = ValueType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  auto inplace_mul_by_inverse_factor_unsafe(FloatType scalar) -> Matrix&;


#if 0
  template <typename U, sint32 Rows_, sint32 Cols_>
  friend class Matrix; // needed to access member data() in operator*=

  template <typename U, sint32 Size, bool isLower>
  friend class TriangularMatrix; // needed to access member data() in solve()
#endif
  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  static constexpr auto TransposedRows() -> sint32 { return IsRowMajor ? Cols : Rows; }
  static constexpr auto TransposedCols() -> sint32 { return IsRowMajor ? Rows : Cols; }

  Storage _data{};
};

//
// non member class operators

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
auto operator+(const Matrix<ValueType, Rows, Cols, IsRowMajor>& m1, const Matrix<ValueType, Rows, Cols, IsRowMajor>& m2)
    -> Matrix<ValueType, Rows, Cols, IsRowMajor>
{
  Matrix<ValueType, Rows, Cols, IsRowMajor> temp{m1};
  return (temp += m2);
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
auto operator-(const Matrix<ValueType, Rows, Cols, IsRowMajor>& m1, const Matrix<ValueType, Rows, Cols, IsRowMajor>& m2)
    -> Matrix<ValueType, Rows, Cols, IsRowMajor>
{
  Matrix<ValueType, Rows, Cols, IsRowMajor> temp{m1};
  return (temp -= m2);
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
auto operator*(const Matrix<ValueType, Rows, Cols, IsRowMajor>& m, ValueType scalar) -> Matrix<ValueType, Rows, Cols, IsRowMajor>
{
  Matrix<ValueType, Rows, Cols, IsRowMajor> temp{m};
  return (temp *= scalar);
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
auto operator*(ValueType scalar, const Matrix<ValueType, Rows, Cols, IsRowMajor>& m) -> Matrix<ValueType, Rows, Cols, IsRowMajor>
{
  return (m * scalar);
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
auto operator/(const Matrix<ValueType, Rows, Cols, IsRowMajor>& m, ValueType scalar)
    -> tl::expected<std::reference_wrapper<Matrix<ValueType, Rows, Cols, IsRowMajor>>,
                    typename Matrix<ValueType, Rows, Cols, IsRowMajor>::Errors>
{
  Matrix<ValueType, Rows, Cols, IsRowMajor> temp{m};
  return (temp /= scalar);
}

//template <typename ValueType, sint32 Rows1, sint32 Cols1, sint32 Cols2>
//auto operator*(const Matrix<ValueType, Rows1, Cols1>& m1, const Matrix<ValueType, Cols1, Cols2>& m2)
//    -> Matrix<ValueType, Rows1, Cols2>
//{
//  Matrix<ValueType, Rows1, Cols1> temp(m1);
//  return (temp *= m2);
//}


#if 0
template <typename ValueType, sint32 Rows, sint32 Cols>
Matrix<ValueType, Rows, Cols>::Matrix(const std::array<ValueType, static_cast<size_t>(Rows* Cols)>& arr)
    : _data{Eigen::Map<const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(arr.data(), Rows, Cols)}
{
}

template <typename ValueType, sint32 Rows, sint32 Cols>
template <sint32 Cols2>
inline auto Matrix<ValueType, Rows, Cols>::operator*=(const Matrix<ValueType, Cols, Cols2>& other)
    -> Matrix<ValueType, Rows, Cols2>
{
  Matrix<ValueType, Rows, Cols2> temp;
  temp._data = (_data * other._data).eval();
  return temp;
}

template <typename ValueType, sint32 Rows, sint32 Cols>
template <sint32 SrcRowSize,
          sint32 SrcColSize,
          sint32 SrcRowCount,
          sint32 SrcColCount,
          sint32 SrcRowBeg,
          sint32 SrcColBeg,
          sint32 DstRowBeg,
          sint32 DstColBeg>
void Matrix<ValueType, Rows, Cols>::setBlock(const Matrix<ValueType, SrcRowSize, SrcColSize>& block)
{
  static_assert((SrcRowCount > 1) && (SrcColCount > 1), "use scalar access operator for block copy size == 1");
  static_assert(SrcRowBeg + SrcRowCount <= SrcRowSize, "copy to many rows from src");
  static_assert(SrcColBeg + SrcColCount <= SrcColSize, "copy to many cols from src");

  static_assert(DstRowBeg + SrcRowCount <= Rows, "copy to many rows to dst");
  static_assert(DstColBeg + SrcColCount <= Cols, "copy to many cols to dst");

  for (sint32 row = 0; row < SrcRowCount; ++row)
  {
    for (sint32 col = 0; col < SrcColCount; ++col)
    {
      this->operator()(DstRowBeg + row, DstColBeg + col) = block(SrcRowBeg + row, SrcColBeg + col);
    }
  }
}

#endif

} // namespace math
} // namespace tracking

#endif // FDEAAACC_9EF1_4C87_94DC_2FA494822664
