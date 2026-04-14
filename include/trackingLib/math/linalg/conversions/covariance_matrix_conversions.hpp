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
/// \return tl::expected containing the CovarianceMatrixFactored instance on success
/// \see CovarianceMatrixFactoredFromList() (overloaded) for nested initializer list input
/// \see CovarianceMatrixFullFromList() for full covariance matrixes
template <typename ValueType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromCovarianceMatrixFull(
    const typename CovarianceMatrixFactored<ValueType_, Size_>::compose_type& other)
    -> tl::expected<CovarianceMatrixFactored<ValueType_, Size_>, Errors>
{
  const auto retVal = other.decomposeUDUT();
  if (retVal.has_value())
  {
    const auto [u, d] = retVal.value_or(std::make_pair(TriangularMatrix<ValueType_, Size_, false, true>::Identity(),
                                                       DiagonalMatrix<ValueType_, Size_>::Identity()));
    return CovarianceMatrixFactored{std::move(u), std::move(d)};
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
/// \return CovarianceMatrixFactored instance with UDU^T decomposition of the input
/// \note The input matrix must be symmetric, otherwise assertion fails
/// \see CovarianceMatrixFactoredFromList() (overloaded) for separate U and D input
/// \see CovarianceMatrixFullFromList() for full covariance matrixes
template <typename ValueType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromList(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> CovarianceMatrixFactored<ValueType_, Size_>
{
  const auto other = CovarianceMatrixFactored<ValueType_, Size_>::compose_type::FromList(list);
  const auto cov   = CovarianceMatrixFactoredFromCovarianceMatrixFull<ValueType_, Size_>(other);
  assert(cov.has_value() && "Input matrix cannot be factored into UDUt form");
  return cov.value();
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72
