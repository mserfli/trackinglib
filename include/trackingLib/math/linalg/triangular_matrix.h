#ifndef BC7FD90F_FBB7_481C_89C4_89BEE41309C5
#define BC7FD90F_FBB7_481C_89C4_89BEE41309C5

#include "base/first_include.h"        // IWYU pragma: keep
#include "math/linalg/square_matrix.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix;

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL;

// TODO(matthias): add interface contract
// TODO(matthias): use optimized menory storage for triangular matrices

/// \brief A triangular matrix specialization that stores only the upper or lower triangular part.
///
/// This class represents triangular matrices (upper or lower) and provides operations
/// optimized for triangular structure. It inherits from SquareMatrix but restricts
/// access to maintain triangular properties. Memory usage is not yet optimized and
/// still stores the full square matrix internally.
///
/// \tparam ValueType_ The data type of matrix elements (e.g., float32, float64)
/// \tparam Size_ The dimension of the triangular matrix (compile-time constant)
/// \tparam IsLower_ Triangular type flag (true for lower triangular, false for upper triangular)
/// \tparam IsRowMajor_ Storage layout flag (true for row-major, false for column-major)
///
/// \note Currently uses full square matrix storage. Future optimization should use
///       triangular storage to save memory (Size_*(Size_+1)/2 elements instead of Size_^2).
///
/// \see SquareMatrix for general square matrix operations
/// \see DiagonalMatrix for diagonal matrix operations
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_ = true>
class TriangularMatrix TEST_REMOVE_FINAL: public SquareMatrix<ValueType_, Size_, IsRowMajor_>
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


  /// \brief Construct an identity triangular matrix.
  ///
  /// Creates a triangular matrix with ones on the diagonal and zeros elsewhere,
  /// maintaining the triangular structure constraint.
  ///
  /// \return TriangularMatrix An identity matrix respecting triangular constraints
  ///
  /// \note For unit triangular matrices, this creates a matrix with ones on the diagonal
  [[nodiscard]] static auto Identity() -> TriangularMatrix { return TriangularMatrix{BaseSquareMatrix::Identity()}; }

  /// \brief Set a triangular block within this triangular matrix.
  ///
  /// Copies a triangular block from the source matrix to the specified position
  /// in this matrix, maintaining triangular structure constraints.
  ///
  /// \tparam SrcSize Size of the source triangular block
  /// \tparam SrcCount Number of diagonal elements to copy
  /// \tparam SrcRowBeg Starting row index in the source block
  /// \tparam SrcColBeg Starting column index in the source block
  /// \tparam DstRowBeg Starting row index in destination (this matrix)
  /// \tparam DstColBeg Starting column index in destination (this matrix)
  /// \param[in] block The source triangular block matrix to copy from
  ///
  /// \note All indices are compile-time constants for efficiency
  template <sint32 SrcSize, sint32 SrcCount, sint32 SrcRowBeg, sint32 SrcColBeg, sint32 DstRowBeg, sint32 DstColBeg>
  void setBlock(const TriangularMatrix<ValueType_, SrcSize, IsLower_, IsRowMajor_>& block);

  // TODO(matthias): add setBlock with params defined at runtime

  /// \brief Multiply triangular matrix with a general matrix.
  ///
  /// Performs matrix multiplication T * M where T is this triangular matrix
  /// and M is a general matrix. Optimized for triangular structure.
  ///
  /// \tparam Cols_ Number of columns in the input matrix
  /// \tparam IsRowMajor2_ Storage layout of the input matrix
  /// \param[in] mat The matrix to multiply with (right-hand side)
  /// \return Matrix The result of the multiplication T * M
  ///
  /// \note Complexity is O(Size_² * Cols_), optimized for triangular operations
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

  /// \brief Solve the triangular system T*x = b using forward/backward substitution.
  ///
  /// Solves the linear system T*x = b where T is this triangular matrix.
  /// Uses efficient forward substitution for lower triangular or backward
  /// substitution for upper triangular matrices.
  ///
  /// \tparam Cols_ Number of columns in the right-hand side matrix b
  /// \tparam IsRowMajor2_ Storage layout of the right-hand side matrix
  /// \param[in] b Right-hand side matrix of the equation T*x = b
  /// \return Matrix Solution matrix x such that T*x ≈ b
  ///
  /// \note Complexity is O(Size_² * Cols_), much faster than general matrix solving
  /// \note Assumes the triangular matrix is non-singular
  template <sint32 Cols_, bool IsRowMajor2_>
  [[nodiscard]] auto solve(const Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>& b) const
      -> Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>;

  /// \brief Compute the inverse of the triangular matrix.
  ///
  /// Calculates the inverse using specialized triangular matrix inversion algorithms.
  /// The result maintains the triangular structure.
  ///
  /// \return TriangularMatrix The inverse matrix such that T * T^(-1) = I
  ///
  /// \note More efficient than general matrix inversion due to triangular structure
  /// \note Assumes the matrix is non-singular
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
