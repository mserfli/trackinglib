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

/// \brief A general-purpose matrix class implementing a Rows x Cols matrix with configurable storage layout.
///
/// This class provides a comprehensive matrix implementation supporting both row-major and column-major
/// storage layouts. It offers arithmetic operations, element access, block operations, and various
/// matrix manipulations. All operations use compile-time dimension checking for type safety.
///
/// Key features:
/// - Configurable row-major or column-major storage layout
/// - Compile-time dimension checking for type safety
/// - Comprehensive arithmetic operations (+, -, *, /) with matrices and scalars
/// - Element access with bounds checking using tl::expected
/// - Block operations for submatrix manipulation
/// - Matrix multiplication with automatic result type deduction
/// - Transpose operations with zero-copy views
/// - Frobenius norm calculation for floating-point types
///
/// \tparam ValueType_ The data type of matrix elements (typically arithmetic types like float32, float64, int)
/// \tparam Rows_ The number of rows in the matrix (must be positive, > 0)
/// \tparam Cols_ The number of columns in the matrix (must be positive, > 0)
/// \tparam IsRowMajor_ Storage layout: true for row-major order, false for column-major order
///
/// \note Row-major storage stores elements consecutively by row in memory (element [i][j] followed by [i][j+1]),
///       while column-major stores by column (element [i][j] followed by [i+1][j]). The choice affects:
///       - Memory access patterns and cache efficiency
///       - Compatibility with external libraries (e.g., Eigen defaults to column-major)
///       - Performance for row-wise vs column-wise operations
///
/// \note Error handling uses the tl::expected<T, Errors> pattern throughout. Operations that can fail
///       (e.g., division by zero, out-of-bounds access) return tl::expected containing either the result
///       or an error descriptor from the Errors enum.
///
/// \note All arithmetic operations support matrices with different storage layouts but same dimensions.
///       The result layout matches the left-hand side operand.
///
/// \see Errors for possible error conditions and their meanings
/// \see Matrix for usage examples below
///
/// \warning Matrix dimensions are fixed at compile time. Dynamic sizing is not supported.
///
/// Example usage:
/// \code{.cpp}
/// // Create a 3x2 row-major matrix of floats
/// auto mat = Matrix<float32, 3, 2, true>::Zeros();
///
/// // Element access with bounds checking
/// auto result = mat(1, 0); // Returns tl::expected<float32, Errors>
/// if (result) {
///     float32 value = *result;
/// } else {
///     // Handle error: result.error() contains Errors enum value
/// }
///
/// // Arithmetic operations
/// auto ones = Matrix<float32, 3, 2, true>::Ones();
/// auto sum = mat + ones; // Element-wise addition
/// auto scaled = mat * 2.0f; // Scalar multiplication
///
/// // Matrix multiplication (3x2) * (2x4) = (3x4)
/// auto other = Matrix<float32, 2, 4, true>::Ones();
/// auto product = mat * other;
///
/// // Transpose (creates view, zero-copy)
/// auto transposed = mat.transpose(); // Type: Matrix<float32, 2, 3, false>
/// \endcode
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class Matrix: public contract::MatrixIntf<Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>, Matrix>
{
public:
  //////////////////////////////////////////////////
  // type definitions  --->
  /// \brief Type of the same matrix with opposite memory layout
  using opposite_mem_layout_type = Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>;
  /// \brief Type of the transposed matrix without changing the memory layout
  using transpose_type = Matrix<ValueType_, Cols_, Rows_, !IsRowMajor_>;
  /// \brief Type of the transposed matrix with fixed row major memory layout
  using transpose_type_row_major = Matrix<ValueType_, Cols_, Rows_, true>;
  using value_type               = ValueType_; ///< element type
  /// \brief number of rows, columns, and memory layout
  static constexpr auto Rows       = Rows_;       ///< number of rows
  static constexpr auto Cols       = Cols_;       ///< number of cols
  static constexpr auto IsRowMajor = IsRowMajor_; ///< memory layout of the matrix
  /// \brief number of rows and columns in memory layout
  static constexpr auto RowsInMem = IsRowMajor_ ? Rows_ : Cols_;
  static constexpr auto ColsInMem = IsRowMajor_ ? Cols_ : Rows_;

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
  /// \brief Element read-only access with bounds checking
  /// \param[in] row  row of the element to read (0-based index)
  /// \param[in] col  column of the element to read (0-based index)
  /// \return tl::expected<ValueType_, Errors>   either the value at (row,col) or an Error descriptor
  /// \note Bounds checking is performed. Returns Errors::OutOfBounds if row < 0 || row >= Rows || col < 0 || col >= Cols
  [[nodiscard]] auto operator()(sint32 row, sint32 col) const -> tl::expected<ValueType_, Errors>;

  /// \brief Element modify access with bounds checking
  /// \param[in] row  row of the element to modify (0-based index)
  /// \param[in] col  column of the element to modify (0-based index)
  /// \return tl::expected<std::reference_wrapper<ValueType_>, Errors>   either the reference at (row,col) or an Error descriptor
  /// \note Bounds checking is performed. Returns Errors::OutOfBounds if row < 0 || row >= Rows || col < 0 || col >= Cols
  [[nodiscard]] auto operator()(sint32 row, sint32 col) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>;
  // <---

  //////////////////////////////////////////////////
  // comparison operators  --->
  /// \brief Comparison to other matrix
  /// \tparam IsRowMajor2_  memory layout of other matrix
  /// \param[in] other  matrix with any memory layout
  /// \return true   if all elements are equal
  template <bool IsRowMajor2_>
  [[nodiscard]] auto operator==(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor2_>& other) const -> bool;

  /// \brief Inequality comparison to other matrix
  /// \tparam IsRowMajor2_  memory layout of other matrix
  /// \param[in] other  matrix with any memory layout
  /// \return true   if any element differs
  template <bool IsRowMajor2_>
  [[nodiscard]] auto operator!=(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor2_>& other) const -> bool;
  // <---

  //////////////////////////////////////////////////
  // arithmetic assignment operators  --->
  /// \brief Calculates Self = Self + Other
  /// \tparam IsRowMajor2_  memory layout of other matrix
  /// \param[in] other  matrix with any memory layout
  template <bool IsRowMajor2_>
  void operator+=(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor2_>& other);

  /// \brief Calculates Self = Self - Other
  /// \tparam IsRowMajor2_  memory layout of other matrix
  /// \param[in] other  matrix with any memory layout
  template <bool IsRowMajor2_>
  void operator-=(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor2_>& other);

  /// \brief Calculates Self = Self * scalar
  /// \param[in] scalar   scalar value
  void operator*=(ValueType_ scalar);

  /// \brief Calculates Self = Self / scalar for integral matrices
  /// \param[in] scalar   integral scalar value
  /// \return tl::expected<void, Errors>   success or divide_by_zero error
  /// \note Division by zero (scalar == 0) returns Errors::DivideByZero
  template <typename IntType = ValueType_, typename std::enable_if_t<std::is_integral<IntType>::value, bool> = true>
  auto operator/=(IntType scalar) -> tl::expected<void, Errors>;

  /// \brief Calculates Self = Self / scalar for floating-point matrices
  /// \param[in] scalar   floating-point scalar value
  /// \return tl::expected<void, Errors>   success or divide_by_zero error
  /// \note Division by zero (scalar == 0.0) returns Errors::DivideByZero
  template <typename FloatType = ValueType_, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  auto operator/=(FloatType scalar) -> tl::expected<void, Errors>;
  // <---

  //////////////////////////////////////////////////
  // arithmentic operators --->
  /// \brief Calculates Self + Other
  /// \tparam IsRowMajor2_  memory layout of other matrix
  /// \param[in] other  matrix with any memory layout
  /// \return Matrix   result of Self + Other
  template <bool IsRowMajor2_>
  [[nodiscard]] auto operator+(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor2_>& other) const -> Matrix;

  /// \brief Calculates Self - Other
  /// \tparam IsRowMajor2_  memory layout of other matrix
  /// \param[in] other  matrix with any memory layout
  /// \return Matrix   result of Self - Other
  template <bool IsRowMajor2_>
  [[nodiscard]] auto operator-(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor2_>& other) const -> Matrix;

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
  /// \tparam IntType  The integral type of the scalar (automatically deduced)
  /// \param[in] scalar  a scalar value
  /// \return tl::expected<Matrix, Errors>   either the result Self / scalar or an Error descriptor
  /// \note Division by zero (scalar == 0) returns Errors::DivideByZero
  template <typename IntType = ValueType_, typename std::enable_if_t<std::is_integral<IntType>::value, bool> = true>
  [[nodiscard]] auto operator/(IntType scalar) const -> tl::expected<Matrix, Errors>;

  /// \brief Calculates Self / scalar for floating-point matrices
  /// \tparam FloatType  The floating-point type of the scalar (automatically deduced)
  /// \param[in] scalar  a scalar value
  /// \return tl::expected<Matrix, Errors>   either the result Self / scalar or an Error descriptor
  /// \note Division by zero (scalar == 0.0) returns Errors::DivideByZero
  template <typename FloatType = ValueType_, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  [[nodiscard]] auto operator/(FloatType scalar) const -> tl::expected<Matrix, Errors>;

  /// \brief Calculates matrix multiplication Self * Other
  /// \tparam Cols2_  Number of columns in the other matrix
  /// \tparam IsRowMajor2_  Storage layout of the other matrix
  /// \param[in] other  Right-hand side matrix for multiplication
  /// \return Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>  Result matrix with dimensions (Rows x Cols2)
  /// \note Matrix multiplication requires that Self.Cols == Other.Rows. The result layout matches the left operand.
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
  /// \return ValueType_   the Frobenius norm of the matrix
  /// \note Only available for floating-point types. For integral types, this method is not instantiated.
  template <typename FloatType = ValueType_, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool> = true>
  [[nodiscard]] auto frobenius_norm() const -> ValueType_;

  /// \brief Fast transpose without changing the layout (zero-copy view)
  /// \return const transpose_type&   const reference to same data as Self, but differently interpreted as transposed
  /// \note This creates a view with swapped dimensions and opposite layout. No data is copied.
  [[nodiscard]] auto transpose() const -> const transpose_type&;

  /// \brief Fast transpose without changing the layout (zero-copy view)
  /// \return transpose_type&   reference to same data as Self, but differently interpreted as transposed
  /// \note This creates a view with swapped dimensions and opposite layout. No data is copied.
  [[nodiscard]] auto transpose() -> transpose_type&;

  /// \brief Fast transpose without changing the layout (rvalue version)
  /// \return transpose_type   rvalue reference to same data as Self, but differently interpreted as transposed
  /// \note This creates a view with swapped dimensions and opposite layout. No data is copied. Use for temporary objects.
  [[nodiscard]] auto transpose_rvalue() && -> transpose_type;

  /// \brief Set a block matrix at given position (compile-time parameters)
  /// \tparam SrcRowSize_      Rows of the source block matrix
  /// \tparam SrcColSize_      Cols of the source block matrix
  /// \tparam SrcRowCount_     Number of rows to copy from source (must be > 0)
  /// \tparam SrcColCount_     Number of cols to copy from source (must be > 0)
  /// \tparam SrcRowBeg_       Begin row index in source (0-based, must be >= 0)
  /// \tparam SrcColBeg_       Begin col index in source (0-based, must be >= 0)
  /// \tparam SrcIsRowMajor_   Memory layout of source matrix
  /// \tparam DstRowBeg_       Begin row index in destination (0-based, must be >= 0)
  /// \tparam DstColBeg_       Begin col index in destination (0-based, must be >= 0)
  /// \param[in] block         Source block matrix to copy from
  /// \note All indices and counts are compile-time constants. The copied region must fit within both matrices. No runtime bounds
  /// checking.
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
  /// \tparam SrcRowSize_      Rows of the source block matrix
  /// \tparam SrcColSize_      Cols of the source block matrix
  /// \tparam SrcIsRowMajor_   Memory layout of source matrix
  /// \param srcRowCount       Number of rows to copy from source (must be > 0)
  /// \param srcColCount       Number of cols to copy from source (must be > 0)
  /// \param srcRowBeg         Begin row index in source (0-based, must be >= 0)
  /// \param srcColBeg         Begin col index in source (0-based, must be >= 0)
  /// \param dstRowBeg         Begin row index in destination (0-based, must be >= 0)
  /// \param dstColBeg         Begin col index in destination (0-based, must be >= 0)
  /// \param[in] block         Source block matrix to copy from
  /// \note The copied region must fit within both source and destination bounds. No bounds checking is performed.
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
  /// \brief Unsafe element read-only access (no bounds checking)
  /// \param[in] row  row of the element to read (0-based index, must be valid)
  /// \param[in] col  column of the element to read (0-based index, must be valid)
  /// \return ValueType_   the value at (row,col)
  /// \warning No bounds checking is performed. Undefined behavior if indices are out of bounds.
  [[nodiscard]] auto at_unsafe(sint32 row, sint32 col) const -> ValueType_;

  /// \brief Unsafe element modify access (no bounds checking)
  /// \param[in] row  row of the element to modify (0-based index, must be valid)
  /// \param[in] col  column of the element to modify (0-based index, must be valid)
  /// \return ValueType_&   reference to the value at (row,col)
  /// \warning No bounds checking is performed. Undefined behavior if indices are out of bounds.
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

/// \brief Calculates Scalar + Matrix (element-wise addition)
/// \tparam ValueType_  Element type of the matrix
/// \tparam Rows_  Number of rows in the matrix
/// \tparam Cols_  Number of columns in the matrix
/// \tparam IsRowMajor_  Storage layout of the matrix
/// \param[in] scalar  Scalar value to add to each element
/// \param[in] mat  Matrix operand
/// \return Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>  Result of scalar + matrix
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
[[nodiscard]] static auto operator+(ValueType_ scalar, const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& mat)
    -> Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>
{
  return mat + scalar;
}

/// \brief Calculates Scalar * Matrix (element-wise multiplication)
/// \tparam ValueType_  Element type of the matrix
/// \tparam Rows_  Number of rows in the matrix
/// \tparam Cols_  Number of columns in the matrix
/// \tparam IsRowMajor_  Storage layout of the matrix
/// \param[in] scalar  Scalar value to multiply with each element
/// \param[in] mat  Matrix operand
/// \return Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>  Result of scalar * matrix
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
