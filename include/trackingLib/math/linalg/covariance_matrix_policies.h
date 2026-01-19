#ifndef D891F0CE_1B63_4381_AA8F_2C4A973E5FF9
#define D891F0CE_1B63_4381_AA8F_2C4A973E5FF9

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"

namespace tracking
{
namespace math
{

/// \brief Base class for covariance matrix policy types
///
/// This empty base class serves as a common ancestor for all covariance matrix policy types.
/// It enables type checking and provides a common interface for policy-based design patterns.
///
/// Policy classes derived from this base must provide:
/// - A `FloatType` type alias defining the floating-point type
/// - A template alias `template <sint32 Size> using Instantiate` for creating covariance matrix instances
/// \see FullCovarianceMatrixPolicy
/// \see FactoredCovarianceMatrixPolicy
struct CovarianceMatrixPolicyBase
{
};

/// \brief Policy for using full covariance matrix representation
///
/// This policy type creates standard full covariance matrices that store the complete
/// symmetric positive definite matrix explicitly. This representation is straightforward
/// and suitable for most applications where numerical stability is not a primary concern.
///
/// \tparam FloatT_ Floating-point type for matrix elements (e.g., float32, float64)
///
/// Usage example:
/// \code
/// using FullPolicy = tracking::math::FullCovarianceMatrixPolicy<float32>;
/// using MyCovarianceMatrix = FullPolicy::Instantiate<6>; // Creates 6x6 full covariance matrix
/// \endcode
///
/// \see CovarianceMatrixFull
/// \see FactoredCovarianceMatrixPolicy
/// \see CovarianceMatrixPolicyBase
template <typename FloatT_>
struct FullCovarianceMatrixPolicy: CovarianceMatrixPolicyBase
{
  /// \brief tag to determine the covariance matrix type
  static constexpr bool is_factored = false;

  /// \brief Floating-point type used for all calculations
  using FloatType = FloatT_;

  /// \brief Template alias for instantiating covariance matrices
  /// \tparam Size Dimension of the covariance matrix (Size × Size)
  /// \return Concrete covariance matrix type for the specified size
  template <sint32 Size>
  using Instantiate = CovarianceMatrixFull<FloatType, Size>;
};

/// \brief Policy for using UDU factored covariance matrix representation
///
/// This policy type creates covariance matrices using UDU factorization, which stores
/// the matrix as U·D·Uᵀ where U is a unit upper triangular matrix and D is a diagonal matrix.
/// This representation provides better numerical stability and is particularly useful for
/// Kalman filtering applications where matrix conditioning can be problematic.
///
/// The UDU factorization is based on academic publications by Bierman, Thornton, and D'Souza,
/// providing numerically stable algorithms for prediction and update operations.
///
/// \tparam FloatT_ Floating-point type for matrix elements (e.g., float32, float64)
///
/// Usage example:
/// \code
/// using FactoredPolicy = tracking::math::FactoredCovarianceMatrixPolicy<float64>;
/// using MyCovarianceMatrix = FactoredPolicy::Instantiate<4>; // Creates 4x4 factored covariance matrix
/// \endcode
///
/// \see CovarianceMatrixFactored
/// \see FullCovarianceMatrixPolicy
/// \see CovarianceMatrixPolicyBase
template <typename FloatT_>
struct FactoredCovarianceMatrixPolicy: CovarianceMatrixPolicyBase
{
  /// \brief tag to determine the covariance matrix type
  static constexpr bool is_factored = true;

  /// \brief Floating-point type used for all calculations
  using FloatType = FloatT_;

  /// \brief Template alias for instantiating covariance matrices
  /// \tparam Size Dimension of the covariance matrix (Size × Size)
  /// \return Concrete covariance matrix type for the specified size
  template <sint32 Size>
  using Instantiate = CovarianceMatrixFactored<FloatType, Size>;
};

} // namespace math
} // namespace tracking

#endif // D891F0CE_1B63_4381_AA8F_2C4A973E5FF9
