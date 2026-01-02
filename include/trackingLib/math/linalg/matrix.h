#ifndef FDEAAACC_9EF1_4C87_94DC_2FA494822664
#define FDEAAACC_9EF1_4C87_94DC_2FA494822664

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/contracts/matrix_intf.h"
#include "math/linalg/errors.h"
#include "math/linalg/matrix_types.h" // IWYU pragma: keep
#include <algorithm>
#include <array>
#include <tuple>
#include <type_traits>

namespace tracking
{
namespace math
{

// TODO(matthias): add support for external memory
// TODO(matthias): add template params to support coord transformations (inFrame e.g. EGO_k_1, outFrame, power -> 2 for covs)
/// Compliant to AUTOSAR C++14, A8-4-8: Output parameters shall not be used, making use of RVO and NRVO

/// \brief  Implements a Row x Col matrix stored internally according to IsRowMajor
/// \tparam ValueType_   element type
/// \tparam Rows_        number of rows
/// \tparam Cols_        number of cols
/// \tparam IsRowMajor_  memory layout of the matrix
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class Matrix: public contract::MatrixIntf<Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>, Matrix>
{
public:
  /// \brief Type of the transposed matrix without changing the memory layout
  using transpose_type = Matrix<ValueType_, Cols_, Rows_, !IsRowMajor_>;
  /// \brief Type of the transposed matrix with fixed row major memory layout
  using transpose_type_row_major   = Matrix<ValueType_, Cols_, Rows_, true>;
  using value_type                 = ValueType_;                  ///< element type
  static constexpr auto Rows       = Rows_;                       ///< number of rows
  static constexpr auto Cols       = Cols_;                       ///< number of cols
  static constexpr auto RowsInMem  = IsRowMajor_ ? Rows_ : Cols_; ///< number of rows in memory
  static constexpr auto ColsInMem  = IsRowMajor_ ? Cols_ : Rows_; ///< number of cols in memory
  static constexpr auto IsRowMajor = IsRowMajor_;                 ///< memory layout of the matrix

  // rule of 5 declarations
  Matrix()                                     = default;
  Matrix(const Matrix&)                        = default;
  Matrix(Matrix&&) noexcept                    = default;
  auto operator=(const Matrix&) -> Matrix&     = default;
  auto operator=(Matrix&&) noexcept -> Matrix& = default;
  virtual ~Matrix()                            = default;

  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a Zero matrix
  /// \return Zero matrix
  [[nodiscard]] static auto Zeros() -> Matrix;

  /// \brief Construct a matrix filled with ones
  /// \return One matrix
  [[nodiscard]] static auto Ones() -> Matrix;
  // <---

  //////////////////////////////////////////////////
  // access operators  --->
  /// \brief Element read-only access
  /// \param[in] row  row of the element to read
  /// \param[in] col  column of the element to read
  /// \return tl::expected<ValueType_, Errors>   either the value at (row,col) or an Error descriptor
  [[nodiscard]] auto operator()(sint32 row, sint32 col) const -> tl::expected<ValueType_, Errors>;

  /// \brief Element modify access
  /// \param[in] row  row of the element to modify
  /// \param[in] col  column of the element to modify
  /// \return tl::expected<std::reference_wrapper<ValueType_>, Errors>   either the reference at (row,col) or an Error descriptor
  [[nodiscard]] auto operator()(sint32 row, sint32 col) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>;
  // <---

  //////////////////////////////////////////////////
  // comparison operators  --->
  /// \brief Comparison to equal matrix type
  /// \param[in] other  matrix of equal type
  /// \return true   if all elements are equal
  [[nodiscard]] auto operator==(const Matrix& other) const -> bool;

  /// \brief Comparison to matrix of equal size, but opposite memory layout
  /// \param[in] other  matrix with opposite memory layout
  /// \return true   if all elements are equal
  [[nodiscard]] auto operator==(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other) const -> bool;

  /// \brief Inequality comparison to equal matrix type
  /// \param[in] other  matrix of equal type
  /// \return true   if any element differs
  [[nodiscard]] auto operator!=(const Matrix& other) const -> bool;

  /// \brief Inequality comparison to matrix of equal size, but opposite memory layout
  /// \param[in] other  matrix with opposite memory layout
  /// \return true   if any element differs
  [[nodiscard]] auto operator!=(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other) const -> bool;
  // <---

  //////////////////////////////////////////////////
  // arithmetic assignment operators  --->
  /// \brief Calculates Self = Self + Other
  /// \param[in] other  matrix of equal size
  void operator+=(const Matrix& other);

  /// \brief Calculates Self = Self + Other
  /// \param[in] other  matrix of equal size, but opposite memory layout
  void operator+=(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other);

  /// \brief Calculates Self = Self - Other
  /// \param[in] other  matrix of equal size
  void operator-=(const Matrix& other);

  /// \brief Calculates Self = Self - Other
  /// \param[in] other  matrix of equal size, but opposite memory layout
  void operator-=(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other);

  /// \brief Calculates Self = Self * scalar
  /// \param[in] scalar   scalar value
  void operator*=(ValueType_ scalar);

  /// \brief Calculates Self = Self / scalar for integral matrices
  /// \param[in] scalar   integral scalar value
  /// \return tl::expected<void, Errors>   success or divide_by_zero error
  template <typename IntType = ValueType_, typename std::enable_if_t<std::is_integral<IntType>::value, bool> = true>
  auto operator/=(IntType scalar) -> tl::expected<void, Errors>;

  /// \brief Calculates Self = Self / scalar for floating-point matrices
  /// \param[in] scalar   floating-point scalar value
  /// \return tl::expected<void, Errors>   success or divide_by_zero error
  template <typename FloatType = ValueType_, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  auto operator/=(FloatType scalar) -> tl::expected<void, Errors>;
  // <---

  //////////////////////////////////////////////////
  // arithmentic operators --->
  /// \brief Calculates Self + Other
  /// \tparam MajorOrder_  memory layout of other matrix
  /// \param[in] other  matrix of equal size
  /// \return Matrix   result of Self + Other
  template <bool MajorOrder_>
  [[nodiscard]] auto operator+(const Matrix<ValueType_, Rows_, Cols_, MajorOrder_>& other) const -> Matrix;

  /// \brief Calculates Self - Other
  /// \tparam MajorOrder_  memory layout of other matrix
  /// \param[in] other  matrix of equal size
  /// \return Matrix   result of Self - Other
  template <bool MajorOrder_>
  [[nodiscard]] auto operator-(const Matrix<ValueType_, Rows_, Cols_, MajorOrder_>& other) const -> Matrix;

  /// \brief Calculates Self + scalar (adds scalar to each element)
  /// \param[in] scalar  a scalar value
  /// \return Matrix   result of Self + scalar
  [[nodiscard]] auto operator+(ValueType_ scalar) const -> Matrix;

  /// \brief Calculates Self - scalar (subtracts scalar from each element)
  /// \param[in] scalar  a scalar value
  /// \return Matrix   result of Self - scalar
  [[nodiscard]] auto operator-(ValueType_ scalar) const -> Matrix;

  /// \brief Calculates Self * scalar
  /// \param[in] scalar  a scalar value
  /// \return Matrix   result of Self * scalar
  [[nodiscard]] auto operator*(ValueType_ scalar) const -> Matrix;

  /// \brief Calculates Self / scalar for integral matrices
  /// \tparam IntType
  /// \param[in] scalar  a scalar value
  /// \return tl::expected<Matrix, Errors>   either the result Self / scalar or an Error descriptor
  template <typename IntType = ValueType_, typename std::enable_if_t<std::is_integral<IntType>::value, bool> = true>
  [[nodiscard]] auto operator/(IntType scalar) const -> tl::expected<Matrix, Errors>;

  /// \brief Calculates Self / scalar for floating-point matrices
  /// \tparam IntType
  /// \param[in] scalar  a scalar value
  /// \return tl::expected<Matrix, Errors>   either the result Self / scalar or an Error descriptor
  template <typename FloatType = ValueType_, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  [[nodiscard]] auto operator/(FloatType scalar) const -> tl::expected<Matrix, Errors>;

  /// \brief Calculates matrix multiplication Self * Other
  /// \tparam Cols2_
  /// \tparam IsRowMajor2_
  /// \param[in] other
  /// \return Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>
  template <sint32 Cols2_, bool IsRowMajor2_>
  [[nodiscard]] auto operator*(const Matrix<ValueType_, Cols_, Cols2_, IsRowMajor2_>& other) const
      -> Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>;
  // <---

  //////////////////////////////////////////////////
  // other operations --->

  /// \brief Sets all elements to zero
  void setZeros();

  /// \brief Sets all elements to one
  void setOnes();

  /// \brief Get min value of the matrix
  /// \return min value
  [[nodiscard]] auto min() const -> ValueType_ { return *std::min_element(data().begin(), data().end()); }

  /// \brief Get max value of the matrix
  /// \return max value
  [[nodiscard]] auto max() const -> ValueType_ { return *std::max_element(data().begin(), data().end()); }

  /// \brief Get min and max value of the matrix
  /// \return tuple of min, max value
  [[nodiscard]] auto minmax() const -> std::tuple<ValueType_, ValueType_>;

  /// \brief Calculate Frobenius norm (L2 norm): sqrt(sum of squared elements)
  /// Only available for floating-point types
  /// \return ValueType_   the Frobenius norm of the matrix
  template <typename FloatType = ValueType_, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  [[nodiscard]] auto frobenius_norm() const -> ValueType_;

  /// \brief Fast transpose without changing the layout
  /// \return const transpose_type&   const reference to same data as Self, but differently interpreted
  [[nodiscard]] auto transpose() const -> const transpose_type&;

  /// \brief Fast transpose without changing the layout
  /// \return transpose_type&   reference to same data as Self, but differently interpreted
  [[nodiscard]] auto transpose() -> transpose_type&;

  /// \brief Fast transpose without changing the layout
  /// \return transpose_type   rvalue to same data as Self, but differently interpreted
  [[nodiscard]] auto transpose_rvalue() && -> transpose_type;

  /// \brief Set a block matrix at given position
  /// \tparam SrcRowSize_      Rows of the source block
  /// \tparam SrcColSize_      Cols of the source block
  /// \tparam SrcRowCount_     Number of rows to copy from source
  /// \tparam SrcColCount_     Number of cols to copy from source
  /// \tparam SrcRowBeg_       Begin row index in source
  /// \tparam SrcColBeg_       Begin col index in source
  /// \tparam SrcIsRowMajor_   Memory layout of source
  /// \tparam DstRowBeg_       Begin row index in dest
  /// \tparam DstColBeg_       Begin col index in dest
  /// \param[in] block         Source block matrix to copy from
  template <sint32 SrcRowSize_,
            sint32 SrcColSize_,
            sint32 SrcRowCount_,
            sint32 SrcColCount_,
            sint32 SrcRowBeg_,
            sint32 SrcColBeg_,
            bool   SrcIsRowMajor_,
            sint32 DstRowBeg_,
            sint32 DstColBeg_>
  void setBlock(const Matrix<ValueType_, SrcRowSize_, SrcColSize_, SrcIsRowMajor_>& block);

  /// \brief Set a block matrix at given position
  /// \tparam SrcRowSize_      Rows of the source block
  /// \tparam SrcColSize_      Cols of the source block
  /// \tparam SrcIsRowMajor_   Memory layout of source
  /// \param srcRowCount       Number of rows to copy from source
  /// \param srcColCount       Number of cols to copy from source
  /// \param srcRowBeg         Begin row index in source
  /// \param srcColBeg         Begin col index in source
  /// \param dstRowBeg         Begin row index in dest
  /// \param dstColBeg         Begin col index in dest
  /// \param[in] block         Source block matrix to copy from
  template <sint32 SrcRowSize_, sint32 SrcColSize_, bool SrcIsRowMajor_>
  void setBlock(const sint32                                                        srcRowCount,
                const sint32                                                        srcColCount,
                const sint32                                                        srcRowBeg,
                const sint32                                                        srcColBeg,
                const sint32                                                        dstRowBeg,
                const sint32                                                        dstColBeg,
                const Matrix<ValueType_, SrcRowSize_, SrcColSize_, SrcIsRowMajor_>& block);
  // <---

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Unsafe element read-only access
  /// \param[in] row  row of the element to read
  /// \param[in] col  column of the element to read
  /// \return ValueType_   the value at (row,col)
  [[nodiscard]] auto at_unsafe(sint32 row, sint32 col) const -> ValueType_;

  /// \brief Unsafe element modify access
  /// \param[in] row  row of the element to read
  /// \param[in] col  column of the element to read
  /// \return ValueType_   the value at (row,col)
  [[nodiscard]] auto at_unsafe(sint32 row, sint32 col) -> ValueType_&;
  // <---

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on
  using Storage = std::array<ValueType_, static_cast<sint32>(Rows* Cols)>; ///< type of the internal storage

  //////////////////////////////////////////////////
  // access operators  --->
  /// \brief  Read-only access to the internal data
  /// \return const Storage&
  auto data() const -> const Storage& { return _data; }

  /// \brief Modify access to the internal data
  /// \return Storage&
  auto data() -> Storage& { return _data; }
  // <---

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  template <typename IntType = ValueType_, typename std::enable_if_t<std::is_integral<IntType>::value, bool> = true>
  void inplace_div_by_int_unsafe(IntType scalar);

  template <typename FloatType = ValueType_, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  void inplace_mul_by_inverse_factor_unsafe(FloatType scalar);

  Storage _data{}; ///< internal data to store the matrix
};

//////////////////////////////////////////////////
// non member operations  --->

/// \brief Calculates Scalar + Matrix
/// \tparam ValueType_
/// \tparam Rows_
/// \tparam Cols_
/// \tparam IsRowMajor_
/// \param[in] scalar
/// \param[in] mat
/// \return Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
[[nodiscard]] static auto operator+(ValueType_ scalar, const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& mat)
    -> Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>
{
  return mat + scalar;
}

/// \brief Calculates Scalar * Matrix
/// \tparam ValueType_
/// \tparam Rows_
/// \tparam Cols_
/// \tparam IsRowMajor_
/// \param[in] scalar
/// \param[in] mat
/// \return Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
[[nodiscard]] static auto operator*(ValueType_ scalar, const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& mat)
    -> Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>
{
  return mat * scalar;
}
// <---

} // namespace math
} // namespace tracking

#endif // FDEAAACC_9EF1_4C87_94DC_2FA494822664
