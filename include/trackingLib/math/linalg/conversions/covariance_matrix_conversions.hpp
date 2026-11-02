#ifndef A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72
#define A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72

#include "conversions.h"
#include "math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep
#include "math/linalg/conversions/square_conversions.hpp"     // IWYU pragma: keep
#include "math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.hpp"         // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"             // IWYU pragma: keep
#include "math/linalg/square_matrix_decompositions.hpp"       // IWYU pragma: keep
#include <initializer_list>

namespace tracking
{
namespace math
{
namespace conversions
{


/// \brief Creates a CovarianceMatrixFactored from its full covariance matrix
///
/// This function constructs a factored covariance matrix by decomposing the input
/// covariance matrix into UDU^T form. The input must be symmetric.
///
/// \tparam ValueType_ The atomic data type of internal elements
/// \tparam Size_ The dimension of the covariance matrix
/// \param[in] other Full covariance matrix to decompose
/// \return tl::expected containing the factored covariance or the decomposition error
/// \note Preserves existing UDU pivot clamping to epsilon; input is not required to be strictly PD.
///       No comprehensive NaN/Inf rejection is provided. noexcept specifies exception behavior only.
/// \see CovarianceMatrixFactoredFromList() for nested initializer list input
/// \see CovarianceMatrixFull::FromList() for full covariance matrices
template <typename ValueType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromCovarianceMatrixFull(
    const typename CovarianceMatrixFactored<ValueType_, Size_>::compose_type& other) noexcept
    -> tl::expected<CovarianceMatrixFactored<ValueType_, Size_>, Errors>
{
  const auto retVal = other.decomposeUDUT();
  if (retVal.has_value())
  {
    const auto& [u, d] = retVal.value();
    return CovarianceMatrixFactored{u, d};
  }
  return tl::unexpected<Errors>{retVal.error()};
}


/// \brief Creates a CovarianceMatrixFactored from a nested initializer list
///
/// This function constructs a factored covariance matrix by decomposing the input
/// covariance matrix into UDU^T form. The input must be symmetric.
///
/// \tparam ValueType_ The atomic data type of internal elements
/// \tparam Size_ The dimension of the covariance matrix
/// \param[in] list Nested initializer list representing the full covariance matrix
/// \return tl::expected containing the factored covariance or a propagated dimension, symmetry, or decomposition error
/// \note Checks dimensions and symmetry before decomposition. The existing UDU
///       algorithm clamps non-positive pivots to epsilon; it does not enforce strict
///       positive definiteness of the input. For example, diag(-1, 1) succeeds here
///       with the first pivot clamped, while full-policy model creation rejects it.
/// \note No comprehensive NaN/Inf check is provided. noexcept describes exception
///       behavior and does not guarantee successful construction or finite numbers.
/// \see CovarianceMatrixFactored::FromList for explicit U/D input without clamping
template <typename ValueType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromList(const std::initializer_list<std::initializer_list<ValueType_>>& list) noexcept
    -> tl::expected<CovarianceMatrixFactored<ValueType_, Size_>, Errors>
{
  const auto other = CovarianceMatrixFactored<ValueType_, Size_>::compose_type::FromList(list);
  if (!other)
  {
    return tl::unexpected<Errors>{other.error()};
  }
  return CovarianceMatrixFactoredFromCovarianceMatrixFull<ValueType_, Size_>(other.value());
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72
