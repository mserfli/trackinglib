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
class TriangularMatrix;

// forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL;

// TODO(matthias): add interface contract

/// \brief A square matrix specialization of the Matrix class providing additional operations
/// specific to square matrices such as decompositions, inverse calculations, symmetry checks,
/// and matrix property validation.
///
/// This class inherits from Matrix<ValueType_, Size_, Size_, IsRowMajor_> and extends it with
/// operations that are only meaningful for square matrices. It supports various matrix
/// decompositions (QR, LLT, LDLT, UDUT), matrix inversion, symmetry operations, and
/// property checking functions for validation and debugging.
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
class SquareMatrix: public Matrix<ValueType_, Size_, Size_, IsRowMajor_>
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

  /// \brief Performs Householder QR decomposition of a square matrix
  ///
  /// Decomposes a square matrix A into the product A = Q*R, where Q is an orthogonal matrix
  /// and R is an upper triangular matrix. This implementation uses Householder reflections
  /// for numerical stability.
  ///
  /// The Householder QR decomposition is based on successive Householder transformations that
  /// zero out subdiagonal elements column by column. Each Householder reflection is represented
  /// by a vector v and a scalar τ, where the reflection matrix is I - τ*v*v^T.
  ///
  /// \return std::pair<SquareMatrix, TriangularMatrix> A pair containing Q and R matrices,
  ///         where Q is orthogonal and R is upper triangular such that A = Q * R
  ///
  /// \note The decomposition satisfies A = Q * R, with Q being orthogonal (Q^T * Q = I)
  ///       and R being upper triangular. This is useful for solving linear systems and
  ///       computing matrix inverses.
  /// \note Time complexity: O(n^3) for an n x n matrix
  /// \note Space complexity: O(n^2) additional space for Q and R matrices
  /// \note Numerically stable and suitable for general square matrices
  ///
  /// \see qrSolve for solving linear systems using this decomposition
  /// \see inverse for matrix inversion using QR decomposition
  /// \see https://www.cs.cornell.edu/~bindel/class/cs6210-f09/lec18.pdf for algorithm reference
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

  /// \brief Performs Cholesky decomposition (LLT) of a symmetric positive definite matrix
  ///
  /// Decomposes a symmetric positive definite matrix A into the product A = L*L^T, where L
  /// is a lower triangular matrix with positive diagonal elements. This is also known as
  /// the Cholesky decomposition.
  ///
  /// The algorithm uses a recursive approach where each column of L is computed using
  /// previous columns. For a symmetric positive definite matrix, the Cholesky factor L
  /// satisfies L*L^T = A, with L being lower triangular.
  ///
  /// \return tl::expected<TriangularMatrix, Errors> The lower triangular matrix L on success,
  ///         or Errors::matrix_not_symmetric if not symmetric, or
  ///         Errors::matrix_not_positive_definite if not positive definite
  ///
  /// \note The matrix must be symmetric and positive definite
  /// \note Time complexity: O(n^3) for an n x n matrix
  /// \note Space complexity: O(n^2) for the result matrix
  /// \note Numerically stable for well-conditioned positive definite matrices
  ///
  /// \warning The input matrix must be symmetric. Use symmetrize() if needed.
  /// \warning Fails if the matrix is not symmetric or not positive definite
  ///
  /// \see decomposeLDLT for a more numerically stable variant
  /// \see isSymmetric for symmetry checking
  [[nodiscard]] auto decomposeLLT() const -> tl::expected<TriangularMatrix<ValueType_, Size_, true, IsRowMajor_>, Errors>;

  /// \brief Performs LDL^T decomposition of a symmetric positive definite matrix
  ///
  /// Decomposes a symmetric positive definite matrix A into the product A = L*D*L^T, where
  /// L is a unit lower triangular matrix (diagonal elements are 1) and D is a diagonal
  /// matrix with positive diagonal elements.
  ///
  /// This decomposition is more numerically stable than LLT for certain matrices and is
  /// particularly useful in Kalman filtering applications where the matrix structure
  /// needs to be maintained. The LDL^T form separates the triangular structure (L) from
  /// the scaling factors (D).
  ///
  /// \return tl::expected<std::pair<TriangularMatrix, DiagonalMatrix>, Errors>
  ///         A pair containing L and D matrices on success, or Errors::matrix_not_symmetric
  ///         if not symmetric, or Errors::matrix_not_positive_definite if not positive definite
  ///
  /// \note The matrix must be symmetric and positive definite
  /// \note Time complexity: O(n^3) for an n x n matrix
  /// \note Space complexity: O(n^2) for L and O(n) for D
  /// \note More numerically stable than LLT for some applications
  /// \note L has unit diagonal (all diagonal elements are 1)
  ///
  /// \warning Fails if the matrix is not symmetric or not positive definite
  ///
  /// \see decomposeLLT for the standard Cholesky decomposition
  /// \see decomposeUDUT for the UDU^T variant
  [[nodiscard]] auto decomposeLDLT() const
      -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, true, IsRowMajor_>, DiagonalMatrix<ValueType_, Size_>>,
                      Errors>;

  /// \brief Performs UDU^T decomposition of a symmetric matrix
  ///
  /// Decomposes a symmetric matrix A into the product A = U*D*U^T, where U is a unit upper
  /// triangular matrix (diagonal elements are 1) and D is a diagonal matrix. This is also
  /// known as the UDU^T factorization.
  ///
  /// This decomposition is particularly important in Kalman filtering for maintaining the
  /// UDU^T form of covariance matrices, which provides better numerical stability than
  /// standard covariance representations. The algorithm works backwards from the last
  /// column to the first, computing U and D simultaneously.
  ///
  /// The implementation is based on modified Cholesky decomposition and includes numerical
  /// safeguards to ensure positive definiteness even for near-singular matrices.
  ///
  /// \return tl::expected<std::pair<TriangularMatrix, DiagonalMatrix>, Errors>
  ///         A pair containing U and D matrices on success, or Errors::matrix_not_symmetric
  ///         if not symmetric
  ///
  /// \note The matrix must be symmetric
  /// \note Time complexity: O(n^3) for an n x n matrix
  /// \note Space complexity: O(n^2) for U and O(n) for D
  /// \note Numerically stable with safeguards for near-singular matrices
  /// \note U has unit diagonal (all diagonal elements are 1)
  /// \note D diagonal elements are clamped to a minimum value for numerical stability
  ///
  /// \warning Fails if the matrix is not symmetric
  ///
  /// \see decomposeLDLT for the lower triangular variant
  /// \see CovarianceMatrixFactored for UDU usage in Kalman filters
  /// \see Grewal & Andrews, Kalman Filtering Theory and Practice Using MATLAB, 4th Edition, Wiley, 2014
  /// \see Gerald J. Bierman, "Factorization Methods for Discrete Sequential Estimation", 1977
  /// \see Catherine L. Thornton, "Triangular Covariance Factorizations for Kalman Filtering", 1976
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

  /// \brief Checks if the matrix is orthogonal within a specified tolerance
  ///
  /// Tests whether the matrix is orthogonal by checking if Q^T * Q = I within tolerance.
  /// An orthogonal matrix satisfies Q^T * Q = I, where I is the identity matrix.
  ///
  /// \param tolerance Tolerance for numerical comparison (default: 1e-6)
  /// \return true if matrix is orthogonal within tolerance, false otherwise
  ///
  /// \note This is useful for validating QR decomposition results
  /// \note Time complexity: O(n^3) due to matrix multiplication
  /// \see householderQR() for QR decomposition that produces orthogonal matrices
  [[nodiscard]] auto isOrthogonal(ValueType_ tolerance = 1e-6) const -> bool;

  /// \brief Checks if the matrix is upper triangular within a specified tolerance
  ///
  /// Tests whether all elements below the main diagonal are zero within tolerance.
  /// An upper triangular matrix has all elements below the diagonal equal to zero.
  ///
  /// \param tolerance Tolerance for numerical comparison (default: 1e-6)
  /// \return true if matrix is upper triangular within tolerance, false otherwise
  ///
  /// \note This is useful for validating decomposition results (QR, UDUT)
  /// \note Time complexity: O(n^2)
  /// \see householderQR() for QR decomposition that produces upper triangular matrices
  [[nodiscard]] auto isUpperTriangular(ValueType_ tolerance = 1e-6) const -> bool;

  /// \brief Checks if the matrix is lower triangular within a specified tolerance
  ///
  /// Tests whether all elements above the main diagonal are zero within tolerance.
  /// A lower triangular matrix has all elements above the diagonal equal to zero.
  ///
  /// \param tolerance Tolerance for numerical comparison (default: 1e-6)
  /// \return true if matrix is lower triangular within tolerance, false otherwise
  ///
  /// \note This is useful for validating decomposition results (LLT, LDLT)
  /// \note Time complexity: O(n^2)
  /// \see decomposeLLT() for LLT decomposition that produces lower triangular matrices
  [[nodiscard]] auto isLowerTriangular(ValueType_ tolerance = 1e-6) const -> bool;

  /// \brief Checks if the matrix has a unit diagonal within a specified tolerance
  ///
  /// Tests whether all diagonal elements are equal to 1 within tolerance.
  /// A unit diagonal matrix has all diagonal elements equal to 1.
  ///
  /// \param tolerance Tolerance for numerical comparison (default: 1e-6)
  /// \return true if all diagonal elements are 1 within tolerance, false otherwise
  ///
  /// \note This is useful for validating decomposition results (LDLT, UDUT)
  /// \note Time complexity: O(n)
  /// \see decomposeLDLT() for LDLT decomposition that produces unit diagonal matrices
  [[nodiscard]] auto hasUnitDiagonal(ValueType_ tolerance = 1e-6) const -> bool;

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
