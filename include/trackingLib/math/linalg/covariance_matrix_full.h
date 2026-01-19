#ifndef DD2B2494_7486_42BA_84E2_32308E26DBBC
#define DD2B2494_7486_42BA_84E2_32308E26DBBC

#include "base/first_include.h"                           // IWYU pragma: keep
#include "math/linalg/contracts/covariance_matrix_intf.h" // IWYU pragma: keep
#include "math/linalg/square_matrix.h"
#include <initializer_list> // IWYU pragma: keep

namespace tracking
{
namespace math
{

/// \brief Full covariance matrix representation for Kalman filtering
///
/// A covariance matrix represents the uncertainty of a random vector in Kalman filtering.
/// This class provides a standard full matrix representation where all elements are stored.
/// Covariance matrices are always symmetric and positive semi-definite.
///
/// Key properties:
/// - Symmetric: P[i,j] = P[j,i] for all i,j
/// - Positive semi-definite: x^T*P*x >= 0 for all vectors x
/// - Represents uncertainty in state estimation
///
/// This implementation uses row-major storage for consistency with other matrix types.
/// All constructors enforce symmetry through assertions.
///
/// \tparam FloatType_ Floating-point type (float32 or float64)
/// \tparam Size_ Dimension of the square covariance matrix
///
/// \see CovarianceMatrixFactored for UDU factored representation
/// \see SquareMatrix for the base matrix functionality
template <typename FloatType_, sint32 Size_>
class CovarianceMatrixFull
    : public SquareMatrix<FloatType_, Size_, true>
    , public contract::CovarianceMatrixIntf<CovarianceMatrixFull<FloatType_, Size_>>
{
public:
  using BaseSquareMatrix = SquareMatrix<FloatType_, Size_, true>; ///< type of the parent class

  using value_type          = FloatType_;
  using compose_type        = CovarianceMatrixFull;
  static constexpr auto dim = Size_;

  // rule of 5 declarations
  CovarianceMatrixFull()                                                   = default;
  CovarianceMatrixFull(const CovarianceMatrixFull&)                        = default;
  CovarianceMatrixFull(CovarianceMatrixFull&&) noexcept                    = default;
  auto operator=(const CovarianceMatrixFull&) -> CovarianceMatrixFull&     = default;
  auto operator=(CovarianceMatrixFull&&) noexcept -> CovarianceMatrixFull& = default;
  virtual ~CovarianceMatrixFull()                                          = default;

  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a new Covariance Matrix Full<FloatType_, Size_> object
  /// \param[in] other A base class object
  explicit CovarianceMatrixFull(const BaseSquareMatrix& other)
      : BaseSquareMatrix{other}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Move construct a new Covariance Matrix Full<FloatType_, Size_> object
  /// \param[in] other A base class object
  explicit CovarianceMatrixFull(BaseSquareMatrix&& other) noexcept
      : BaseSquareMatrix{std::move(other)}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Construct a new Covariance Matrix Full<FloatType_, Size_> object from a transposed SquareMatrix
  /// \param[in] other A transposed base class object
  explicit CovarianceMatrixFull(const typename BaseSquareMatrix::transpose_type& other)
      : BaseSquareMatrix{other.transpose()}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Move construct a new Covariance Matrix Full<FloatType_, Size_> object from a transposed SquareMatrix
  /// \param[in] other A transposed base class object
  explicit CovarianceMatrixFull(typename BaseSquareMatrix::transpose_type&& other) noexcept
      : BaseSquareMatrix{std::move(other).transpose_rvalue()}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFull  Resulting identity matrix
  static auto Identity() -> CovarianceMatrixFull { return CovarianceMatrixFull{BaseSquareMatrix::Identity()}; }

  /// \brief Construct a diagonal covariance matrix
  /// \return CovarianceMatrixFactored
  static auto FromDiagonal(const DiagonalMatrix<FloatType_, Size_>& diag) -> CovarianceMatrixFull;

  /// \brief Creates a CovarianceMatrixFull from a nested initializer list
  ///
  /// This function constructs a full covariance matrix from a nested initializer list.
  ///
  /// \param[in] list Nested initializer list representing the covariance matrix
  /// \return CovarianceMatrixFull instance initialized with the provided values
  static auto FromList(const std::initializer_list<std::initializer_list<FloatType_>>& list) -> CovarianceMatrixFull
  {
    return CovarianceMatrixFull{BaseSquareMatrix::FromList(list)};
  }
  // <---

  /// \brief Set internal matrix to the Identity matrix
  void setIdentity() { BaseSquareMatrix::setIdentity(); }

  /// \brief Access operator to the covariance value at (row, col)
  using BaseSquareMatrix::operator();

  /// \brief Creates the "composed" covariance, although no composition is needed
  /// \return const CovarianceMatrixFull&
  auto operator()() const -> const CovarianceMatrixFull& { return *this; }

  /// \brief Calculate the determinant of the covariance matrix.
  ///
  /// \return ValueType_ The determinant of the matrix
  /// \note uses SquareMatrix::determinant
  auto determinant() const -> FloatType_ { return BaseSquareMatrix::determinant(); }

  /// \brief Calculates the matrix inverse using Cholesky decomposition
  ///
  /// Computes the inverse of the covariance matrix using Cholesky decomposition.
  /// This method is numerically stable for positive definite matrices.
  ///
  /// \return tl::expected containing the inverse matrix on success, or an error code
  /// \return Errors::NotPositiveDefinite if the matrix is not positive definite
  ///
  /// \note Time complexity: O(n^3) where n = Size_
  /// \note The result satisfies: (*this) * result = Identity()
  auto inverse() const -> tl::expected<CovarianceMatrixFull, Errors>;

  /// \brief Compute inverse as full covariance matrix
  ///
  /// Alias for inverse() to match the contract definitions
  ///
  /// \return Full covariance matrix containing the inverse
  auto composed_inverse() const -> tl::expected<compose_type, Errors> { return this->inverse(); }

  /// \brief Compute A*P*A^T in-place (covariance propagation)
  ///
  /// Performs the matrix operation A*P*A^T where P is this covariance matrix.
  /// This operation propagates covariance through a linear transformation A.
  /// The result overwrites the current matrix.
  ///
  /// \tparam IsRowMajor_ Storage layout of the transformation matrix A
  /// \param[in] A Square transformation matrix
  ///
  /// \note This is a key operation in Kalman filter prediction
  /// \note Time complexity: O(n^3) where n = Size_
  template <bool IsRowMajor_>
  void apaT(const tracking::math::SquareMatrix<FloatType_, Size_, IsRowMajor_>& A);

  /// \brief Compute A*P*A^T (covariance propagation)
  ///
  /// Performs the matrix operation A*P*A^T where P is this covariance matrix.
  /// This operation propagates covariance through a linear transformation A.
  /// Returns a new covariance matrix without modifying this one.
  ///
  /// \tparam IsRowMajor_ Storage layout of the transformation matrix A
  /// \param[in] A Square transformation matrix
  /// \return New covariance matrix containing A*P*A^T
  ///
  /// \note This is a key operation in Kalman filter prediction
  /// \note Time complexity: O(n^3) where n = Size_
  template <bool IsRowMajor_>
  auto apaT(const tracking::math::SquareMatrix<FloatType_, Size_, IsRowMajor_>& A) const -> CovarianceMatrixFull;

  /// \brief Set diagonal variance element and clear correlations
  ///
  /// Sets the variance (diagonal element) at position (idx, idx) to the given value
  /// and clears all off-diagonal correlations involving this state variable.
  /// This effectively makes the idx-th state variable uncorrelated with others.
  ///
  /// \param[in] idx Zero-based index of the state variable (0 <= idx < Size_)
  /// \param[in] val New variance value (must be positive)
  ///
  /// \note This operation modifies both the diagonal and off-diagonal elements
  /// \note Commonly used to reinitialize uncertainty for specific state components
  void setVariance(const sint32 idx, const FloatType_ val);
};

} // namespace math
} // namespace tracking

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
