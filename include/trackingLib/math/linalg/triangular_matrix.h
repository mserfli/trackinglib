#ifndef BC7FD90F_FBB7_481C_89C4_89BEE41309C5
#define BC7FD90F_FBB7_481C_89C4_89BEE41309C5

#include "base/first_include.h"        // IWYU pragma: keep
#include "math/linalg/square_matrix.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL; // LCOV_EXCL_LINE

// TODO(matthias): add interface contract
// TODO(matthias): use own memory optimized to required number of elements
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_ = true>
class TriangularMatrix TEST_REMOVE_FINAL: public SquareMatrix<ValueType_, Size_, IsRowMajor_> // LCOV_EXCL_LINE
{
public:
  using BaseSquareMatrix = SquareMatrix<ValueType_, Size_, IsRowMajor_>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using BaseSquareMatrix::BaseSquareMatrix;

  /// \brief Type of the transposed matrix without changing the memory layout
  using transpose_type = TriangularMatrix<ValueType_, Size_, !IsLower_, !IsRowMajor_>;

  /// \brief Construct a new Triangular Matrix object
  /// \param[in] other
  explicit TriangularMatrix(const BaseSquareMatrix& other);

  /// \brief Move construct a new Triangular Matrix object
  /// \param[in] other
  explicit TriangularMatrix(BaseSquareMatrix&& other) noexcept
      : BaseSquareMatrix{std::move(other)} {}; // TODO(matthias): might be dangerous due to memory artifacts


  /// \brief Construct an Identity matrix
  /// \return TriangularMatrix  Resulting identity matrix
  [[nodiscard]] static auto Identity() -> TriangularMatrix { return TriangularMatrix{BaseSquareMatrix::Identity()}; }

  /// \brief Set a lower/upper triangular block matrix at given position
  /// \tparam SrcSize    Size of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcRowBeg  Begin row index in source
  /// \tparam SrcColBeg  Begin col index in source
  /// \tparam DstRowBeg  Begin row index in dest
  /// \tparam DstColBeg  Begin col index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount, sint32 SrcRowBeg, sint32 SrcColBeg, sint32 DstRowBeg, sint32 DstColBeg>
  void setBlock(const TriangularMatrix<ValueType_, SrcSize, IsLower_, IsRowMajor_>& block);

  // TODO(matthias): add setBlock with params defined at runtime

  /// \brief Multiplication with generic matrix: Tria * Matrix
  /// \tparam Cols_
  /// \tparam IsRowMajor2_
  /// \param[in] mat
  /// \return Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>
  template <sint32 Cols_, bool IsRowMajor2_>
  [[nodiscard]] auto operator*(const Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>& mat) const
      -> Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>;

  /// \brief Multiplication with triangular matrix: Tria * Matrix
  /// \param[in] mat  A triangular matrix
  /// \return TriangularMatrix<FloatType, Size, isLower>
  [[nodiscard]] auto operator*(const TriangularMatrix& mat) const -> TriangularMatrix;

  /// \brief Multiplication with triangular matrix: Tria * Matrix
  /// \param[in] mat  A triangular matrix
  /// \return SquareMatrix<FloatType, Size>
  [[nodiscard]] auto operator*(const TriangularMatrix<ValueType_, Size_, !IsLower_, IsRowMajor_>& mat) const -> BaseSquareMatrix;

  /// \brief Multiplication with diagonal matrix: Tria * Matrix
  /// \param[in] diag  A diagonal matrix
  /// \return TriangularMatrix<FloatType, Size, isLower>
  [[nodiscard]] auto operator*(const DiagonalMatrix<ValueType_, Size_>& diag) const -> TriangularMatrix;

  /// \brief Multiplication with scalar: Tria * scalar
  /// \param[in] scalar  A scalar value
  /// \return TriangularMatrix<FloatType, Size, isLower>
  [[nodiscard]] auto operator*(const ValueType_ scalar) const -> TriangularMatrix;

  /// \brief Inplace Multiplication with scalar: Tria * scalar
  /// \param[in] scalar  A scalar value
  void operator*=(const ValueType_ scalar);

  /// \brief Element read-only access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType  scalar triangular value
  [[nodiscard]] auto operator()(sint32 row, sint32 col) const -> tl::expected<ValueType_, Errors>;

  /// \brief Element access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType&  Reference to the scalar triangular value
  [[nodiscard]] auto operator()(sint32 row, sint32 col) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>;

  /// \brief Calculate the transposed matrix without changing the layout
  /// \return const transpose_type&   const reference to same data as Self, but differently interpreted
  [[nodiscard]] auto transpose() const -> const transpose_type&;

  /// \brief Calculate the transposed matrix without changing the layout
  /// \return transpose_type&   reference to same data as Self, but differently interpreted
  [[nodiscard]] auto transpose() -> transpose_type&;

  /// \brief Solver for A*x=b based on Cholesky decomposition of A
  /// \tparam Cols_  Number of columns in the rhs variable b
  /// \tparam IsRowMajor2_
  /// \param[in] b  Matrix describing the rhs of the equation A*x=b
  /// \return Matrix<FloatType, Size, Cols, IsRowMajor2_> describing x
  template <sint32 Cols_, bool IsRowMajor2_>
  [[nodiscard]] auto solve(const Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>& b) const
      -> Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>;

  /// \brief Calculates the inverse of the underlying matrix
  /// \return TriangularMatrix
  [[nodiscard]] auto inverse() const -> TriangularMatrix;

  // TODO(matthias): UnitUpper inplace inverse, Grewal Table 6.7 p.235

  /// \brief Checks for Unit Upper condition
  /// \return true
  [[nodiscard]] auto isUnitUpperTriangular() const -> bool;

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Element read-only access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType  scalar triangular value
  [[nodiscard]] auto at_unsafe(sint32 row, sint32 col) const -> ValueType_;

  /// \brief Element access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType&  Reference to the scalar triangular value
  [[nodiscard]] auto at_unsafe(sint32 row, sint32 col) -> ValueType_&;
  // <---

private:
  /// \brief hide inherited transpose function
  using BaseSquareMatrix::transpose;

  /// \brief hide inherited operator() to prevent accessing off-triangular elements
  using BaseSquareMatrix::operator();
};

} // namespace math
} // namespace tracking

#endif // BC7FD90F_FBB7_481C_89C4_89BEE41309C5
