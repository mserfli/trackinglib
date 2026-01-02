#ifndef EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
#define EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL; // LCOV_EXCL_LINE


/// \brief Modified Gram-Schmidt orthogonalization for UDU factorization
///
/// This class implements Catherine Thornton's modified weighted Gram-Schmidt
/// algorithm for computing UDU factorizations in Kalman filtering applications.
/// The algorithm is specifically designed for numerical stability when dealing
/// with covariance matrix updates and predictions.
///
/// The Modified Gram-Schmidt process orthogonalizes the columns of a matrix
/// while maintaining the UDU factorization structure. This is crucial for:
/// - Time propagation (prediction) in Kalman filters
/// - Maintaining numerical stability with ill-conditioned matrices
/// - Efficient computation of matrix square roots in factored form
///
/// The algorithm processes columns from right to left, ensuring that the
/// resulting U matrix is unit upper triangular and D is diagonal, such that
/// A = U*D*U' for the original matrix A.
///
/// \tparam FloatType_ Floating-point type (float32 or float64)
/// \tparam Size_ Matrix dimension (compile-time constant)
///
/// \note Based on Thornton (1976) and Grewal & Andrews (2014)
/// \note Specifically designed for Kalman filter covariance propagation
/// \note Maintains numerical stability through weighted orthogonalization
/// \see CovarianceMatrixFactored for UDU factorization usage
/// \see Rank1Update for rank-1 modifications to factorizations
template <typename FloatType_, sint32 Size_>
class ModifiedGramSchmidt
{
public:
  /// \brief Computes UDU factorization of Phi matrix
  ///
  /// Performs modified Gram-Schmidt orthogonalization to compute the UDU
  /// factorization of the transition matrix Phi. This is used for time
  /// propagation in Kalman filtering where the covariance evolves as
  /// P_k = Phi * P_{k-1} * Phi'.
  ///
  /// The algorithm transforms Phi into U*D*U' form where U is unit upper
  /// triangular and D is diagonal, representing the square root factorization
  /// of Phi * P_{k-1} * Phi'.
  ///
  /// \param[in,out] u Unit upper triangular matrix U (output)
  /// \param[in,out] d Diagonal matrix D (output)
  /// \param[in] Phi State transition matrix (input)
  ///
  /// \note Phi must be row-major (IsRowMajor_ = true)
  /// \note u is initialized to identity and modified in-place
  /// \note Diagonal elements are clipped to prevent numerical instability
  static void run(TriangularMatrix<FloatType_, Size_, false, true>& u,
                  DiagonalMatrix<FloatType_, Size_>&                d,
                  const SquareMatrix<FloatType_, Size_, true>&      Phi);

  /// \brief Computes UDU factorization with process noise
  ///
  /// Performs modified Gram-Schmidt orthogonalization for the complete
  /// Kalman filter prediction step, including both state transition and
  /// process noise contributions. The covariance update becomes:
  /// P_k = Phi * P_{k-1} * Phi' + G * Q * G'.
  ///
  /// This combines the transition matrix Phi with the process noise matrix
  /// G and diagonal noise covariance Q into a single UDU factorization.
  ///
  /// \tparam SizeQ_ Dimension of process noise (compile-time constant)
  /// \param[in,out] u Unit upper triangular matrix U (output)
  /// \param[in,out] d Diagonal matrix D (output)
  /// \param[in] Phi State transition matrix
  /// \param[in] G Process noise coupling matrix
  /// \param[in] Q Diagonal process noise covariance matrix
  ///
  /// \note All matrices must be row-major (IsRowMajor_ = true)
  /// \note u is initialized to identity and modified in-place
  /// \note Diagonal elements are clipped to ensure positive semi-definiteness
  template <sint32 SizeQ_>
  static void run(TriangularMatrix<FloatType_, Size_, false, true>& u,
                  DiagonalMatrix<FloatType_, Size_>&                d,
                  const SquareMatrix<FloatType_, Size_, true>&      Phi,
                  const Matrix<FloatType_, Size_, SizeQ_, true>&    G,
                  const DiagonalMatrix<FloatType_, SizeQ_>&         Q);
};

} // namespace math
} // namespace tracking

#endif // EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
