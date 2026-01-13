#ifndef E23E9AC4_E199_4264_B0DB_0DE4B42F3447
#define E23E9AC4_E199_4264_B0DB_0DE4B42F3447

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix;

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix;

template <typename ValueType_, sint32 Size_>
class Vector;

/// \brief Rank-1 update algorithms for UDU and LDL matrix factorizations
///
/// This class provides static methods for performing rank-1 updates on matrix
/// factorizations commonly used in Kalman filtering. The algorithms maintain
/// the numerical stability of UDU and LDL' factorizations when applying
/// rank-1 modifications of the form P ± x*x' to the original matrix P.
///
/// Rank-1 updates are essential in Kalman filtering for:
/// - Measurement updates (Joseph form updates)
/// - Covariance matrix modifications
/// - Maintaining positive semi-definiteness
///
/// The implementation follows established numerical methods from Gill et al.
/// and ensures that diagonal elements remain positive to preserve matrix
/// conditioning.
///
/// \tparam FloatType_ Floating-point type (float32 or float64)
/// \tparam Size_ Matrix dimension (compile-time constant)
/// \tparam IsRowMajor_ Storage layout (true for row-major, false for column-major)
///
/// \note All operations are performed in-place for efficiency
/// \note Diagonal elements are clipped to prevent numerical instability
/// \see CovarianceMatrixFactored for UDU factorization usage
/// \see ModifiedGramSchmidt for related factorization algorithms
template <typename FloatType_, sint32 Size_, bool IsRowMajor_>
class Rank1Update
{
public:
  /// \brief Performs rank-1 update on UDU factorization (upper triangular form)
  ///
  /// Updates the UDU factorization of a covariance matrix with a rank-1 modification.
  /// This implements the algorithm for updating P = U*D*U' where P is modified by
  /// a rank-1 update of the form P ± x*x'.
  ///
  /// The update preserves the UDU factorization numerically, which is crucial for
  /// maintaining numerical stability in Kalman filtering applications.
  ///
  /// \tparam FloatType_ Floating-point type for matrix elements
  /// \tparam Size_ Matrix dimension (compile-time constant)
  /// \tparam IsRowMajor_ Storage layout flag
  /// \param[in,out] u Upper triangular matrix U in UDU factorization
  /// \param[in,out] d Diagonal matrix D in UDU factorization
  /// \param[in] c Update scalar (positive for update, negative for downdate)
  /// \param[in] x Update vector for rank-1 modification
  ///
  /// \note Based on Gill, Golub, Murray and Saunders (1974) algorithm
  /// \note Ensures positive semi-definiteness by clipping diagonal elements
  /// \see run() for LDL factorization variant
  static void run(TriangularMatrix<FloatType_, Size_, false, IsRowMajor_>& u,
                  DiagonalMatrix<FloatType_, Size_>&                       d,
                  FloatType_                                               c,
                  Vector<FloatType_, Size_>                                x);

  /// \brief Performs rank-1 update on LDL factorization (lower triangular form)
  ///
  /// Updates the LDL' factorization of a covariance matrix with a rank-1 modification.
  /// This implements the algorithm for updating P = L*D*L' where P is modified by
  /// a rank-1 update of the form P ± x*x'.
  ///
  /// The update preserves the LDL' factorization numerically, which is crucial for
  /// maintaining numerical stability in Kalman filtering applications.
  ///
  /// \tparam FloatType_ Floating-point type for matrix elements
  /// \tparam Size_ Matrix dimension (compile-time constant)
  /// \tparam IsRowMajor_ Storage layout flag
  /// \param[in,out] l Lower triangular matrix L in LDL' factorization
  /// \param[in,out] d Diagonal matrix D in LDL' factorization
  /// \param[in] c Update scalar (positive for update, negative for downdate)
  /// \param[in] x Update vector for rank-1 modification
  ///
  /// \note Based on Gill, Golub, Murray and Saunders (1974) algorithm
  /// \note Handles both rank-1 updates and downdates
  /// \see run() for UDU factorization variant
  static void run(TriangularMatrix<FloatType_, Size_, true, IsRowMajor_>& l,
                  DiagonalMatrix<FloatType_, Size_>&                      d,
                  FloatType_                                              c,
                  Vector<FloatType_, Size_>                               x);
};

} // namespace math
} // namespace tracking

#endif // E23E9AC4_E199_4264_B0DB_0DE4B42F3447
