#ifndef F9044FD7_A3A8_43F4_BDD6_F43011384722
#define F9044FD7_A3A8_43F4_BDD6_F43011384722

#include "base/first_include.h"                           // IWYU pragma: keep
#include "math/linalg/contracts/covariance_matrix_intf.h" // IWYU pragma: keep
#include <initializer_list>
#include <iostream> // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix;

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix;

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix;

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix;

template <typename ValueType_, sint32 Size_>
class Vector;

template <typename FloatType_, sint32 Size_>
class CovarianceMatrixFull;

/// \brief UDU factored covariance matrix for numerical stability
///
/// This class implements the UDU^T factorization of covariance matrices, where
/// P = U*D*U^T. Here U is a unit upper triangular matrix (diagonal elements = 1)
/// and D is a diagonal matrix containing the diagonal elements of the factorization.
///
/// UDU factorization provides superior numerical stability compared to full matrix
/// representations, especially for ill-conditioned covariance matrices. This is
/// particularly important in Kalman filtering where covariance matrices can become
/// near-singular over time.
///
/// Key algorithms implemented:
/// - Thornton update: P = Φ*P*Φ^T + G*Q*G^T (prediction step)
/// - Agee-Turner rank-1 update: P = P ± x*x^T (measurement updates)
///
/// The factorization maintains the symmetry and positive semi-definiteness of
/// the covariance matrix while avoiding matrix inversion operations.
///
/// \tparam FloatType_ Floating-point type (float32 or float64)
/// \tparam Size_ Dimension of the square covariance matrix
///
/// \see CovarianceMatrixFull for standard matrix representation
/// \see Thornton, C. L. "Triangular Covariance Factorizations for Kalman Filtering"
/// \see Bierman, G. J. "Factorization Methods for Discrete Sequential Estimation"
///
/// \note Based on academic publications by Thornton, Bierman, and D'Souza
template <typename FloatType_, sint32 Size_>
class CovarianceMatrixFactored //: public contract::CovarianceMatrixIntf<CovarianceMatrixFactored<FloatType_, Size_>>
{
public:
  using value_type          = FloatType_;
  using compose_type        = CovarianceMatrixFull<FloatType_, Size_>;
  static constexpr auto dim = Size_;

  // rule of 5 declarations
  CovarianceMatrixFactored()                                                       = default;
  CovarianceMatrixFactored(const CovarianceMatrixFactored&)                        = default;
  CovarianceMatrixFactored(CovarianceMatrixFactored&&) noexcept                    = default;
  auto operator=(const CovarianceMatrixFactored&) -> CovarianceMatrixFactored&     = default;
  auto operator=(CovarianceMatrixFactored&&) noexcept -> CovarianceMatrixFactored& = default;
  virtual ~CovarianceMatrixFactored()                                              = default;

  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a new Covariance Matrix Factored object
  /// \param[in] u   Unit upper triangular matrix
  /// \param[in] d   Diagonal matrix
  explicit CovarianceMatrixFactored(const TriangularMatrix<FloatType_, Size_, false, true>& u,
                                    const DiagonalMatrix<FloatType_, Size_>&                d);

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFactored
  static auto Identity() -> CovarianceMatrixFactored;

  /// \brief Construct a diagonal covariance matrix
  /// \param[in] diag  Diagonal matrix
  /// \return CovarianceMatrixFactored
  static auto FromDiagonal(const DiagonalMatrix<FloatType_, Size_>& diag) -> CovarianceMatrixFactored;

  /// \brief Creates a CovarianceMatrixFactored from separate U and D initializer lists
  ///
  /// This function constructs a factored covariance matrix from separate initializer lists
  /// for the upper triangular U matrix and diagonal D matrix components.
  ///
  /// \param[in] u Nested initializer list for the upper triangular U matrix
  /// \param[in] d Flat initializer list for the diagonal D matrix
  /// \return CovarianceMatrixFactored instance with the specified U and D components
  static auto FromList(const std::initializer_list<std::initializer_list<FloatType_>>& u,
                       const std::initializer_list<FloatType_>&                        d) -> CovarianceMatrixFactored;
  // <---

  /// \brief Set Identity covariance
  void setIdentity();

  /// \brief Access operator to the covariance value at (row, col)
  /// \param[in,out] row  The specified row
  /// \param[in,out] col  The specified column
  /// \return tl::expected<ValueType_, Errors>   either the value at (row,col) or an Error descriptor
  auto operator()(sint32 row, sint32 col) const -> tl::expected<value_type, Errors>;

  /// \brief Creates the composed covariance
  /// \return compose_type
  auto operator()() const -> compose_type;

  /// \brief Calculate the determinant of the covariance matrix.
  ///
  /// Computes the determinant as the product of the diagonal elements.
  ///     det(cov) = det(U)*det(D)*det(U') = det(D)
  ///
  /// \return ValueType_ The determinant of the matrix
  /// \note Time complexity: O(n) where n is the matrix dimension
  /// \note For singular matrices, the determinant will be zero or very close to zero
  auto determinant() const -> value_type { return _d.determinant(); }

  /// \brief Compute inverse in factored form
  ///
  /// Calculates the inverse of the covariance matrix while maintaining
  /// the UDU factorization. This is more numerically stable than
  /// inverting the composed full matrix.
  ///
  /// \return tl::expected containing the inverse in factored form on success
  /// \return Errors::NotPositiveDefinite if the matrix is singular or near-singular
  ///
  /// \note Time complexity: O(n^3) where n = Size_
  /// \note More stable than full matrix inversion for ill-conditioned covariances
  auto inverse() const -> tl::expected<CovarianceMatrixFactored, Errors>;

  /// \brief Compute inverse as full covariance matrix
  ///
  /// Calculates the inverse and returns it as a full covariance matrix.
  /// This composes the inverse of UDU factorization into a full matrix.
  ///
  /// \return Full covariance matrix containing the inverse
  ///
  /// \note Time complexity: O(n^3) where n = Size_
  auto composed_inverse() const -> compose_type;

  /// \brief Compute A*P*A^T in-place (factored covariance propagation)
  ///
  /// Performs the matrix operation A*P*A^T where P is this factored covariance matrix.
  /// The operation is performed in-place, overwriting the current matrix.
  /// This maintains the UDU factorization throughout the computation.
  ///
  /// \tparam IsRowMajor_ Storage layout of the transformation matrix A
  /// \param[in] A Square transformation matrix
  ///
  /// \note Used for coordinate transformations in factored Kalman filters
  /// \note Maintains numerical stability of the UDU representation
  template <bool IsRowMajor_>
  void apaT(const SquareMatrix<FloatType_, Size_, IsRowMajor_>& A);

  /// \brief Compute A*P*A^T (factored covariance propagation)
  ///
  /// Performs the matrix operation A*P*A^T where P is this factored covariance matrix.
  /// Returns a new factored covariance matrix without modifying this one.
  ///
  /// \tparam IsRowMajor_ Storage layout of the transformation matrix A
  /// \param[in] A Square transformation matrix
  /// \return New factored covariance matrix containing A*P*A^T
  ///
  /// \note Used for coordinate transformations in factored Kalman filters
  /// \note Maintains numerical stability of the UDU representation
  template <bool IsRowMajor_>
  auto apaT(const SquareMatrix<FloatType_, Size_, IsRowMajor_>& A) const -> CovarianceMatrixFactored;

  /// \brief Thornton update: Φ*P*Φ^T + G*Q*G^T (Kalman prediction)
  ///
  /// Implements Thornton's algorithm for updating the factored covariance matrix
  /// during the Kalman filter prediction step. This computes:
  /// P_{k+1} = Φ*P_k*Φ^T + G*Q*G^T
  ///
  /// Where:
  /// - Φ is the state transition matrix
  /// - P_k is the current covariance (this matrix)
  /// - G is the process noise coupling matrix
  /// - Q is the diagonal process noise covariance
  ///
  /// The algorithm maintains the UDU factorization throughout the computation,
  /// providing better numerical stability than full matrix operations.
  ///
  /// \tparam SizeQ_ Dimension of the process noise
  /// \param[in] Phi State transition matrix (Size_ x Size_)
  /// \param[in] G Process noise coupling matrix (Size_ x SizeQ_)
  /// \param[in] Q Diagonal process noise covariance (SizeQ_ x SizeQ_)
  ///
  /// \note This is the primary prediction update in factored Kalman filters
  /// \note Time complexity: O(n^2) where n = Size_
  /// \note Numerically stable for ill-conditioned matrices
  ///
  /// \see Thornton, C. L. "Triangular Covariance Factorizations for Kalman Filtering"
  template <sint32 SizeQ_>
  void thornton(const SquareMatrix<FloatType_, Size_, true>&   Phi,
                const Matrix<FloatType_, Size_, SizeQ_, true>& G,
                const DiagonalMatrix<FloatType_, SizeQ_>&      Q);

  /// \brief Agee-Turner rank-1 update: P ± x*x^T
  ///
  /// Performs a rank-1 update of the covariance matrix: P = P + c*x*x^T
  /// where c is a scalar and x is a vector. This operation is fundamental
  /// to Kalman filter measurement updates.
  ///
  /// When c > 0: rank-1 update (increases uncertainty)
  /// When c < 0: rank-1 downdate (decreases uncertainty)
  ///
  /// The algorithm maintains the UDU factorization and is numerically stable.
  ///
  /// \param[in] c Scalar multiplier (positive for update, negative for downdate)
  /// \param[in] x Update vector defining the outer product x*x^T
  ///
  /// \note Used in Kalman measurement updates and Joseph stabilized updates
  /// \note Maintains positive semi-definiteness when c >= -1/λ_min where λ_min is the smallest eigenvalue
  /// \note Time complexity: O(n^2) where n = Size_
  ///
  /// \see Agee, W. H. and Turner, R. C. "Triangular Decomposition of a Positive Definite Matrix Plus a Symmetric Dyad"
  void rank1Update(const FloatType_ c, const Vector<FloatType_, Size_>& x);

  /// \brief Set the variance at (idx,idx) and clears any correlations
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setVariance(const sint32 idx, const FloatType_ val);

  /// \brief Fill the covariance with first N=SrcCount rows and cols of the other covariance
  /// \tparam SrcSize   Size of the other covariance
  /// \tparam SrcCount  Count rows/cols to copy from other
  /// \param[in] other  The other matrix to copy from
  template <sint32 SrcSize_, sint32 SrcCount_>
  void fill(const CovarianceMatrixFactored<FloatType_, SrcSize_>& other);

  /// \brief Set the Diagonal matrix element to given value
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setDiagonal(const sint32 idx, const FloatType_ val);

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Unsafe element read-only access
  /// \param[in] row  row of the element to read
  /// \param[in] col  column of the element to read
  /// \return ValueType_   the value at (row,col)
  [[nodiscard]] auto at_unsafe(sint32 row, sint32 col) const -> FloatType_;
  /// <---

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  TriangularMatrix<FloatType_, Size_, false, true> _u{};
  DiagonalMatrix<FloatType_, Size_>                _d{};
};

} // namespace math
} // namespace tracking

#endif // F9044FD7_A3A8_43F4_BDD6_F43011384722
