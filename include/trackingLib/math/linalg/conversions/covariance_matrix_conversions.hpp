#ifndef A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72
#define A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72

#include "conversions.h"
#include "math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep
#include "math/linalg/conversions/square_conversions.hpp"     // IWYU pragma: keep
#include "math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.hpp"         // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"             // IWYU pragma: keep
#include <initializer_list>

namespace tracking
{
namespace math
{
namespace conversions
{

/// \brief Creates a CovarianceMatrixFactored from separate U and D initializer lists
///
/// This function constructs a factored covariance matrix from separate initializer lists
/// for the upper triangular U matrix and diagonal D matrix components.
///
/// \tparam FloatType_ The floating point type for matrix elements
/// \tparam Size_ The dimension of the covariance matrix
/// \param[in] u Nested initializer list for the upper triangular U matrix
/// \param[in] d Flat initializer list for the diagonal D matrix
/// \return CovarianceMatrixFactored instance with the specified U and D components
/// \see CovarianceMatrixFactoredFromList() (overloaded) for single list input
/// \see CovarianceMatrixFullFromList() for full covariance matrices
template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromList(const std::initializer_list<std::initializer_list<FloatType_>>& u,
                                             const std::initializer_list<FloatType_>&                        d)
    -> CovarianceMatrixFactored<FloatType_, Size_>
{
  auto&& u_ = TriangularFromList<FloatType_, Size_, false, true>(u);
  auto&& d_ = DiagonalFromList<FloatType_, Size_>(d);
  return CovarianceMatrixFactored{std::move(u_), std::move(d_)};
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
  const auto other = SquareFromList<FloatType_, Size_, true>(list);
  assert(other.isSymmetric() && "Input matrix is not symmetric");
  const auto retVal = other.decomposeUDUT();
  assert(retVal.has_value());
  const auto [u, d] = retVal.value_or(std::make_pair(TriangularMatrix<FloatType_, Size_, false, true>::Identity(),
                                                     DiagonalMatrix<FloatType_, Size_>::Identity()));
  return CovarianceMatrixFactored{std::move(u), std::move(d)};
#endif
}

/// \brief Creates a CovarianceMatrixFull from a nested initializer list
///
/// This function constructs a full covariance matrix from a nested initializer list.
///
/// \tparam FloatType_ The floating point type for matrix elements
/// \tparam Size_ The dimension of the covariance matrix
/// \param[in] list Nested initializer list representing the covariance matrix
/// \return CovarianceMatrixFull instance initialized with the provided values
/// \see CovarianceMatrixFactoredFromList() for factored covariance matrices
/// \see CovarianceMatrixFromList() for generic covariance matrix creation
template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFullFromList(const std::initializer_list<std::initializer_list<FloatType_>>& list)
    -> CovarianceMatrixFull<FloatType_, Size_>
{
  return CovarianceMatrixFull{SquareFromList<FloatType_, Size_, true>(list)};
}

/// \brief Creates a CovarianceMatrix from a nested initializer list (generic version)
///
/// This template function provides a unified interface for creating covariance matrices
/// from nested initializer lists. It automatically dispatches to the appropriate
/// conversion function based on the covariance matrix type (full or factored).
///
/// \tparam CovarianceMatrixType Template template parameter specifying the covariance type
/// \tparam FloatType_ The floating point type for matrix elements
/// \tparam Size_ The dimension of the covariance matrix
/// \param[in] list Nested initializer list representing the covariance matrix
/// \return CovarianceMatrix instance of the specified type
/// \see CovarianceMatrixFullFromList() for full covariance matrices
/// \see CovarianceMatrixFactoredFromList() for factored covariance matrices
template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType, typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFromList(const std::initializer_list<std::initializer_list<FloatType_>>& list)
    -> CovarianceMatrixType<FloatType_, Size_>
{
  // Use the specific conversion functions based on the covariance matrix type
  if constexpr (std::is_same_v<CovarianceMatrixType<FloatType_, Size_>, CovarianceMatrixFull<FloatType_, Size_>>)
  {
    return CovarianceMatrixFullFromList<FloatType_, Size_>(list);
  }
  else if constexpr (std::is_same_v<CovarianceMatrixType<FloatType_, Size_>, CovarianceMatrixFactored<FloatType_, Size_>>)
  {
    return CovarianceMatrixFactoredFromList<FloatType_, Size_>(list);
  }
  else
  {
    // This should not happen for valid covariance matrix types
    static_assert(!std::is_same_v<CovarianceMatrixType<FloatType_, Size_>, CovarianceMatrixType<FloatType_, Size_>>,
                  "Unsupported covariance matrix type");
  }
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // A170ADCF_D2AE_4DF6_B6B6_D3DF0FA46A72
