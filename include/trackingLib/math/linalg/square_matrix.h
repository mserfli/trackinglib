#ifndef BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
#define BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/errors.h" // IWYU pragma: keep
#include "math/linalg/matrix.h" // IWYU pragma: keep
#include <initializer_list>
#include <utility> // std::pair

namespace tracking
{
namespace math
{

// forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix;

// forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix;

// TODO(matthias): add interface contract

/// \brief A square matrix specialization of the Matrix class providing additional operations
/// specific to square matrixes such as decompositions, inverse calculations, symmetry checks,
/// and matrix property validation.
///
/// This class inherits from Matrix<ValueType_, Size_, Size_, IsRowMajor_> and extends it with
/// operations that are only meaningful for square matrixes. It supports various matrix
/// decompositions (QR, LLT, LDLT, UDUT), matrix inversion, symmetry operations, and
/// property checking functions for validation and debugging.
///
/// \tparam ValueType_ The atomic data type of internal elements
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
  /// \brief Construct a new Square Matrix<ValueType_, Size_> object
  /// \param[in] other A base class object
  explicit SquareMatrix(const BaseMatrix& other)
      : BaseMatrix{other}
  {
  }

  /// \brief Move construct a new Square Matrix<ValueType_, Size_> object
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
  /// \note This constructor is defined in the implementation file to avoid circular dependencies
  SquareMatrix(const DiagonalMatrix<ValueType_, Size_>& other); // NOLINT(google-explicit-constructor)


  /// \brief Construct an identity matrix with ones on the diagonal and zeros elsewhere.
  ///
  /// \return SquareMatrix An identity matrix of the specified size and storage layout
  ///
  /// \note The identity matrix I satisfies I * A = A * I = A for any square matrix A
  [[nodiscard]] static auto Identity() -> SquareMatrix;

  /// \brief Creates a SquareMatrix from a nested initializer list
  ///
  /// This function constructs a SquareMatrix from a nested initializer list where each inner list
  /// represents a row of the matrix. The dimensions must be square and match the template parameter.
  ///
  /// \tparam ValueType_ The atomic data type of internal elements
  /// \tparam Size_ The dimension of the square matrix
  /// \tparam IsRowMajor_ The storage layout (true for row-major, false for column-major)
  /// \param[in] list Nested initializer list in logical row-major format
  /// \return SquareMatrix instance initialized with the provided values
  /// \throws std::runtime_error If the list dimensions don't match the square matrix size
  /// \see SquareFromDiagonal() for creating from diagonal matrixes
  /// \see MatrixFromList() for general matrix creation
  [[nodiscard]] static auto FromList(const std::initializer_list<std::initializer_list<ValueType_>>& list) -> SquareMatrix;

  /// \brief Set the matrix to the identity matrix in-place.
  ///
  /// Modifies the current matrix to have ones on the diagonal and zeros elsewhere.
  ///
  /// \note This operation modifies the matrix in-place and does not change its size or layout
  void setIdentity();

  /// \brief Fast transpose without changing the layout (zero-copy view)
  /// \return const transpose_type&   const reference to same data as Self, but differently interpreted as transposed
  /// \note This creates a view with swapped dimensions and opposite layout. No data is copied.
  [[nodiscard]] auto transpose() const -> const SquareMatrix<ValueType_, Size_, !IsRowMajor_>&
  {
    return static_cast<const SquareMatrix<ValueType_, Size_, !IsRowMajor_>&>(BaseMatrix::transpose());
  };

  /// \brief Fast transpose without changing the layout (zero-copy view)
  /// \return transpose_type&   reference to same data as Self, but differently interpreted as transposed
  /// \note This creates a view with swapped dimensions and opposite layout. No data is copied.
  [[nodiscard]] auto transpose() -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>&
  {
    return static_cast<SquareMatrix<ValueType_, Size_, !IsRowMajor_>&>(BaseMatrix::transpose());
  };

  /// \brief Calculate the trace of the square matrix.
  ///
  /// Computes the sum of all diagonal elements of the matrix.
  /// The trace is defined as the sum of elements A_ii for i = 1 to n.
  ///
  /// \return ValueType_ The trace of the matrix(sum of diagonal elements)
  [[nodiscard]] auto trace() const -> ValueType_;

  /// \brief Calculate the determinant of the square matrix.
  ///
  /// Computes the determinant using LU decomposition with partial pivoting.
  /// The determinant is the product of the diagonal elements of the upper triangular
  /// matrix from the LU decomposition, multiplied by (-1)^k where k is the number
  /// of row permutations.
  ///
  /// \return ValueType_ The determinant of the matrix
  /// \note Time complexity: O(n^3) where n is the matrix dimension
  /// \note For singular matrixes, the determinant will be zero or very close to zero
  /// \note Uses partial pivoting for numerical stability
  [[nodiscard]] auto determinant() const -> ValueType_;

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
  /// \return std::pair<SquareMatrix, TriangularMatrix> A pair containing Q and R matrixes,
  ///         where Q is orthogonal and R is upper triangular such that A = Q * R
  ///
  /// \note The decomposition satisfies A = Q * R, with Q being orthogonal (Q^T * Q = I)
  ///       and R being upper triangular. This is useful for solving linear systems and
  ///       computing matrix inverses.
  /// \note Time complexity: O(n^3) for an n x n matrix
  /// \note Space complexity: O(n^2) additional space for Q and R matrixes
  /// \note Numerically stable and suitable for general square matrixes
  ///
  /// \see qrSolve for solving linear systems using this decomposition
  /// \see inverse for matrix inversion using QR decomposition
  /// \see https://www.cs.cornell.edu/~bindel/class/cs6210-f09/lec18.pdf for algorithm reference
  [[nodiscard]] auto householderQR() const -> std::pair<SquareMatrix, TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>>;

  /// \brief Solve the linear system A * x = b using QR decomposition.
  ///
  /// Uses the QR decomposition of this matrix to solve for x in A * x = b.
  /// This method is numerically stable and suitable for well-conditioned matrixes.
  ///
  /// \tparam IsRowMajor2_ The row-majority of the right-hand side matrix b
  /// \param[in] b The right-hand side matrix of the equation A * x = b
  /// \return SquareMatrix The solution matrix x such that A * x ≈ b
  ///
  /// \note This method internally performs QR decomposition, which has O(n³) complexity.
  ///       For multiple right-hand sides with the same A, consider using householderQR()
  ///       directly and reusing the decomposition.
  ///
  /// \see householderQR for the underlying decomposition
  template <bool IsRowMajor2_>
  [[nodiscard]] auto qrSolve(const SquareMatrix<ValueType_, Size_, IsRowMajor2_>& b) const
      -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>;

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
  /// \note Numerically stable for well-conditioned positive definite matrixes
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
  /// This decomposition is more numerically stable than LLT for certain matrixes and is
  /// particularly useful in Kalman filtering applications where the matrix structure
  /// needs to be maintained. The LDL^T form separates the triangular structure (L) from
  /// the scaling factors (D).
  ///
  /// \return tl::expected<std::pair<TriangularMatrix, DiagonalMatrix>, Errors>
  ///         A pair containing L and D matrixes on success, or Errors::matrix_not_symmetric
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
  /// UDU^T form of covariance matrixes, which provides better numerical stability than
  /// standard covariance representations. The algorithm works backwards from the last
  /// column to the first, computing U and D simultaneously.
  ///
  /// The implementation is based on modified Cholesky decomposition and includes numerical
  /// safeguards to ensure positive definiteness even for near-singular matrixes.
  ///
  /// \return tl::expected<std::pair<TriangularMatrix, DiagonalMatrix>, Errors>
  ///         A pair containing U and D matrixes on success, or Errors::matrix_not_symmetric
  ///         if not symmetric
  ///
  /// \note The matrix must be symmetric
  /// \note Time complexity: O(n^3) for an n x n matrix
  /// \note Space complexity: O(n^2) for U and O(n) for D
  /// \note Numerically stable with safeguards for near-singular matrixes
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
  /// \warning Fails for singular or near-singular matrixes. Check condition number if needed.
  ///
  /// \see householderQR for the underlying decomposition
  [[nodiscard]] auto inverse() const -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>;

  /// \brief Symmetrize the matrix by averaging with its transpose.
  ///
  /// Modifies the matrix to be symmetric by computing (A + A^T) / 2.
  /// This is useful for ensuring symmetry when small numerical errors
  /// have made a theoretically symmetric matrix asymmetric.
  ///
  /// \note This operation modifies the matrix in-place and ensures perfect symmetry
  ///       at the cost of slightly changing the original values.
  void symmetrize();

  //////////////////////////////////////////////////
  // square matrix property checks  --->
  /// \brief Check if the matrix is symmetric.
  ///
  /// Tests whether the matrix equals its transpose (A = A^T).
  ///
  /// \param tolerance Tolerance for numerical comparison (default: 1e-6)
  /// \return true if the matrix is symmetric, false otherwise
  ///
  /// \note Symmetry checking uses element-wise comparison and may have floating-point precision issues
  [[nodiscard]] auto isSymmetric(ValueType_ tolerance = 1e-6) const -> bool;

  /// \brief Check if the diagonal matrix is positive definite.
  ///
  /// A diagonal matrix is positive definite if all diagonal elements are positive.
  ///
  /// \return true if all diagonal elements are > 0, false otherwise
  ///
  /// \note For diagonal matrixes, positive definiteness is equivalent to all elements > 0
  [[nodiscard]] auto isPositiveDefinite() const -> bool;

  /// \brief Checks if the matrix is positive semi-definite.
  ///
  /// Tests whether all eigenvalues of the matrix are non-negative.
  /// A positive semi-definite matrix satisfies x^T * A * x >= 0 for all non-zero vectors x.
  ///
  /// This is a key property for covariance matrixes in statistics and machine learning.
  ///
  /// The check is performed using Cholesky decomposition; if it succeeds, the matrix
  /// is positive semi-definite.
  ///
  /// \return true if the matrix is positive semi-definite, false otherwise
  [[nodiscard]] auto isPositiveSemiDefinite() const -> bool;

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
  /// \see householderQR() for QR decomposition that produces orthogonal matrixes
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
  /// \see householderQR() for QR decomposition that produces upper triangular matrixes
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
  /// \see decomposeLLT() for LLT decomposition that produces lower triangular matrixes
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
  /// \see decomposeLDLT() for LDLT decomposition that produces unit diagonal matrixes
  [[nodiscard]] auto hasUnitDiagonal(ValueType_ tolerance = 1e-6) const -> bool;
  // <---

private:
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
