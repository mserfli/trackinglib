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
/// \tparam FloatType_ The floating point type for matrix elements
/// \tparam Size_ The dimension of the covariance matrix
/// \param[in] other Full covariance matrix to decompose
/// \return tl::expected containing the CovarianceMatrixFactored instance on success
/// \see CovarianceMatrixFactoredFromList() (overloaded) for nested initializer list input
/// \see CovarianceMatrixFullFromList() for full covariance matrices
template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromCovarianceMatrixFull(
    const typename CovarianceMatrixFactored<FloatType_, Size_>::compose_type& other)
    -> tl::expected<CovarianceMatrixFactored<FloatType_, Size_>, Errors>
{
  const auto retVal = other.decomposeUDUT();
  if (retVal.has_value())
  {
    const auto [u, d] = retVal.value_or(std::make_pair(TriangularMatrix<FloatType_, Size_, false, true>::Identity(),
                                                       DiagonalMatrix<FloatType_, Size_>::Identity()));
    return CovarianceMatrixFactored{std::move(u), std::move(d)};
  }
  return tl::unexpected<Errors>{retVal.error()};
}


/// \brief Creates a CovarianceMatrixFactored from a nested initializer list
///
/// This function constructs a factored covariance matrix by decomposing the input
/// covariance matrix into UDU^T form. The input must be symmetric.
///
/// \tparam FloatType_ The floating point type for matrix elements
/// \tparam Size_ The dimension of the covariance matrix
/// \param[in] list Nested initializer list representing the full covariance matrix
/// \return CovarianceMatrixFactored instance with UDU^T decomposition of the input
/// \note The input matrix must be symmetric, otherwise assertion fails
/// \see CovarianceMatrixFactoredFromList() (overloaded) for separate U and D input
/// \see CovarianceMatrixFullFromList() for full covariance matrices
template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromList(const std::initializer_list<std::initializer_list<FloatType_>>& list)
    -> CovarianceMatrixFactored<FloatType_, Size_>
{
#if 0 // TODO(matthias): check why this is not producing the same results
  // Information Formulation of the UDU Kalman Filter
  // Christopher D’Souza and Renato Zanetti (2018)
  // https://sites.utexas.edu/renato/files/2018/05/UDU_Information.pdf
  const auto other = SquareMatrix<FloatType_, Size_, false>{compose_type::SquareMatrix::FromList(list).transpose()};
  // we use the transpose of the input matrix to get a column major matrix
  // transposing requires a symmetric input matrix
  assert(other.isSymmetric() && "Input matrix is not symmetric");
  // we decompose the input matrix into LDLt form, with L being a column major lower triangular matrix
  const auto retVal = other.decomposeLDLT();
  assert(retVal.has_value());
  const auto [l, d] = retVal.value_or(std::make_pair(TriangularMatrix<FloatType_, Size_, true, false>::Identity(),
                                                     DiagonalMatrix<FloatType_, Size_>::Identity()));
  // we calc the inverse of L and D and map the inv(L).transpose() to U being again a row major upper triangular matrix
  // the resulting UDUt matrix describes the covariance matrix in information form, i.e. the inverse covariance matrix
  return CovarianceMatrixFactored{std::move(l.inverse().transpose()), std::move(d.inverse())};
#else
  const auto other = CovarianceMatrixFactored<FloatType_, Size_>::compose_type::FromList(list);
  const auto cov   = CovarianceMatrixFactoredFromCovarianceMatrixFull<FloatType_, Size_>(other);
  assert(cov.has_value() && "Input matrix cannot be factored into UDUt form");
  return cov.value();
#endif
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72
