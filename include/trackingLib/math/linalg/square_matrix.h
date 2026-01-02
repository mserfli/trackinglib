#ifndef BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
#define BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/errors.h" // IWYU pragma: keep
#include "math/linalg/matrix.h" // IWYU pragma: keep
#include <utility>              // std::pair

namespace tracking
{
namespace math
{

// forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix; // LCOV_EXCL_LINE

// forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL; // LCOV_EXCL_LINE

// TODO(matthias): add interface contract

/// \brief A square matrix specialization of the Matrix class providing additional operations
/// specific to square matrices such as decompositions, inverse calculations, and symmetry checks.
///
/// This class inherits from Matrix<ValueType_, Size_, Size_, IsRowMajor_> and extends it with
/// operations that are only meaningful for square matrices. It supports various matrix
/// decompositions (QR, LLT, LDLT, UDUT), matrix inversion, and symmetry operations.
///
/// \tparam ValueType_ The data type of matrix elements (e.g., float32, float64)
/// \tparam Size_ The dimension of the square matrix (compile-time constant)
/// \tparam IsRowMajor_ Storage layout flag (true for row-major, false for column-major)
///
/// \note All operations maintain the square property and may change storage layout for
///       optimal performance (e.g., inverse operations may toggle IsRowMajor_)
///
/// \see Matrix for the base matrix functionality
/// \see TriangularMatrix for triangular matrix operations
/// \see DiagonalMatrix for diagonal matrix operations
template <typename ValueType_, sint32 Size_, bool IsRowMajor_ = true>
class SquareMatrix: public Matrix<ValueType_, Size_, Size_, IsRowMajor_> // LCOV_EXCL_LINE
{
public:
  using BaseMatrix = Matrix<ValueType_, Size_, Size_, IsRowMajor_>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using BaseMatrix::BaseMatrix;

  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a new Square Matrix<FloatType_, Size_> object
  /// \param[in] other A base class object
  explicit SquareMatrix(const BaseMatrix& other)
      : BaseMatrix{other}
  {
  }

  /// \brief Move construct a new Square Matrix<FloatType_, Size_> object
  /// \param[in] other A base class object
  explicit SquareMatrix(BaseMatrix&& other) noexcept
      : BaseMatrix{std::move(other)}
  {
  }

  /// \brief Construct a square matrix from a diagonal matrix.
  ///
  /// Creates a square matrix with the diagonal elements from the input diagonal matrix
  /// and zeros elsewhere.
  ///
  /// \param[in] other The diagonal matrix to convert from
  ///
  /// \note This is an implicit conversion constructor for convenience
  SquareMatrix(const DiagonalMatrix<ValueType_, Size_>& other); // NOLINT(google-explicit-constructor)


  /// \brief Construct an identity matrix with ones on the diagonal and zeros elsewhere.
  ///
  /// \return SquareMatrix An identity matrix of the specified size and storage layout
  ///
  /// \note The identity matrix I satisfies I * A = A * I = A for any square matrix A
  [[nodiscard]] static auto Identity() -> SquareMatrix;

  /// \brief Set the matrix to the identity matrix in-place.
  ///
  /// Modifies the current matrix to have ones on the diagonal and zeros elsewhere.
  ///
  /// \note This operation modifies the matrix in-place and does not change its size or layout
  void setIdentity();

  /// \brief Perform QR decomposition using Householder transformations.
  ///
  /// Decomposes the matrix A into the product Q * R, where Q is an orthogonal matrix
  /// and R is an upper triangular matrix. This implementation uses Householder reflections
  /// for numerical stability.
  ///
  /// \return std::pair<SquareMatrix, TriangularMatrix> A pair containing Q and R matrices,
  ///         where Q is orthogonal and R is upper triangular such that A = Q * R
  ///
  /// \note The decomposition satisfies A = Q * R, with Q being orthogonal (Q^T * Q = I)
  ///       and R being upper triangular. This is useful for solving linear systems and
  ///       computing matrix inverses.
  ///
  /// \see qrSolve for solving linear systems using this decomposition
  /// \see inverse for matrix inversion using QR decomposition
  [[nodiscard]] auto householderQR() const -> std::pair<SquareMatrix, TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>>;

  /// \brief Solve the linear system A * x = b using QR decomposition.
  ///
  /// Uses the QR decomposition of this matrix to solve for x in A * x = b.
  /// This method is numerically stable and suitable for well-conditioned matrices.
  ///
  /// \param[in] b The right-hand side matrix of the equation A * x = b
  /// \return SquareMatrix The solution matrix x such that A * x ≈ b
  ///
  /// \note This method internally performs QR decomposition, which has O(n³) complexity.
  ///       For multiple right-hand sides with the same A, consider using householderQR()
  ///       directly and reusing the decomposition.
  ///
  /// \see householderQR for the underlying decomposition
  [[nodiscard]] auto qrSolve(const SquareMatrix& b) const -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>;

  /// \brief Perform Cholesky decomposition L*L^T for symmetric positive definite matrices.
  ///
  /// Decomposes a symmetric positive definite matrix A into L*L^T, where L is a lower
  /// triangular matrix. This is the standard Cholesky factorization.
  ///
  /// \return tl::expected<TriangularMatrix, Errors> The lower triangular matrix L on success,
  ///         or an error if the matrix is not symmetric positive definite
  ///
  /// \note The matrix must be symmetric and positive definite
  ///
  /// \note Cholesky decomposition requires O(n³) operations and is numerically stable
  ///       for positive definite matrices. It fails if the matrix has negative eigenvalues
  ///       or is not symmetric.
  ///
  /// \warning The input matrix must be symmetric. Use symmetrize() if needed.
  ///
  /// \see decomposeLDLT for a more robust decomposition
  /// \see isSymmetric for symmetry checking
  [[nodiscard]] auto decomposeLLT() const -> tl::expected<TriangularMatrix<ValueType_, Size_, true, IsRowMajor_>, Errors>;

  /// \brief Perform LDL^T decomposition for symmetric matrices.
  ///
  /// Decomposes a symmetric matrix A into L*D*L^T, where L is a unit lower triangular
  /// matrix (ones on diagonal) and D is a diagonal matrix. This is more robust than
  /// standard Cholesky for positive semi-definite matrices.
  ///
  /// \return tl::expected<std::pair<TriangularMatrix, DiagonalMatrix>, Errors>
  ///         A pair containing L and D matrices on success, or an error if decomposition fails
  ///
  /// \note The matrix must be symmetric
  ///
  /// \note LDL^T decomposition is more numerically stable than LLT for matrices that are
  ///       positive semi-definite or near-singular. The L matrix has unit diagonal (all ones).
  ///
  /// \see decomposeLLT for standard Cholesky decomposition
  /// \see decomposeUDUT for upper triangular variant
  [[nodiscard]] auto decomposeLDLT() const
      -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, true, IsRowMajor_>, DiagonalMatrix<ValueType_, Size_>>,
                      Errors>;

  /// \brief Perform UDU^T decomposition for symmetric matrices.
  ///
  /// Decomposes a symmetric matrix A into U*D*U^T, where U is a unit upper triangular
  /// matrix (ones on diagonal) and D is a diagonal matrix. This is the upper triangular
  /// variant of LDL^T decomposition.
  ///
  /// \return tl::expected<std::pair<TriangularMatrix, DiagonalMatrix>, Errors>
  ///         A pair containing U and D matrices on success, or an error if decomposition fails
  ///
  /// \note The matrix must be symmetric
  ///
  /// \note UDU^T decomposition provides the same benefits as LDL^T but with upper triangular
  ///       structure. Both are used in Kalman filtering for numerical stability.
  ///
  /// \see decomposeLDLT for the lower triangular variant
  /// \see CovarianceMatrixFactored for UDU usage in Kalman filters
  [[nodiscard]] auto decomposeUDUT() const
      -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>, DiagonalMatrix<ValueType_, Size_>>,
                      Errors>;

  /// \brief Compute the matrix inverse using QR decomposition.
  ///
  /// Calculates the inverse of the matrix using QR decomposition for numerical stability.
  /// The result has toggled storage layout for optimal performance.
  ///
  /// \return SquareMatrix The inverse matrix such that A * A^(-1) = I
  ///
  /// \note Uses QR decomposition internally, which provides good numerical stability
  ///       but has O(n³) complexity. The storage layout is toggled in the result.
  ///
  /// \warning Fails for singular or near-singular matrices. Check condition number if needed.
  ///
  /// \see householderQR for the underlying decomposition
  [[nodiscard]] auto inverse() const -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>;

  /// \brief Check if the matrix is symmetric.
  ///
  /// Tests whether the matrix equals its transpose (A = A^T).
  ///
  /// \return true if the matrix is symmetric, false otherwise
  ///
  /// \note Symmetry checking uses element-wise comparison and may have floating-point precision issues
  [[nodiscard]] auto isSymmetric() const -> bool;

  /// \brief Symmetrize the matrix by averaging with its transpose.
  ///
  /// Modifies the matrix to be symmetric by computing (A + A^T) / 2.
  /// This is useful for ensuring symmetry when small numerical errors
  /// have made a theoretically symmetric matrix asymmetric.
  ///
  /// \note This operation modifies the matrix in-place and ensures perfect symmetry
  ///       at the cost of slightly changing the original values.
  void symmetrize();

protected:
  /// \brief Check if all diagonal elements are strictly positive.
  ///
  /// Tests whether all diagonal elements satisfy d_ii > 0.
  /// This is used internally for positive definiteness checks.
  ///
  /// \return true if all diagonal elements are positive, false otherwise
  ///
  /// \note This is a necessary but not sufficient condition for positive definiteness
  [[nodiscard]] auto hasStrictlyPositiveDiagonalElems() const -> bool;
};

} // namespace math
} // namespace tracking
#endif // BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
