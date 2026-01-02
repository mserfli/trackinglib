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

// CovarianceMatrixFactoredFromList: CovarianceMatrixFactored from initializer_list<initializer_list<ValueType_>> and
// initializer_list<ValueType_>
template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactoredFromList(const std::initializer_list<std::initializer_list<FloatType_>>& u,
                                             const std::initializer_list<FloatType_>&                        d)
    -> CovarianceMatrixFactored<FloatType_, Size_>
{
  auto&& u_ = TriangularFromList<FloatType_, Size_, false, true>(u);
  auto&& d_ = DiagonalFromList<FloatType_, Size_>(d);
  return CovarianceMatrixFactored{std::move(u_), std::move(d_)};
}

// CovarianceMatrixFactoredFromList: CovarianceMatrixFactored from initializer_list<initializer_list<ValueType_>>
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

// CovarianceMatrixFullFromList: CovarianceMatrixFull from initializer_list<initializer_list<ValueType_>>
template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFullFromList(const std::initializer_list<std::initializer_list<FloatType_>>& list)
    -> CovarianceMatrixFull<FloatType_, Size_>
{
  return CovarianceMatrixFull{SquareFromList<FloatType_, Size_, true>(list)};
}

/// \brief Create CovarianceMatrix from nested initializer list (generic version)
/// \tparam CovarianceMatrixType  Template template parameter for covariance type
/// \tparam FloatType_            Floating point type
/// \tparam Size_                 Matrix dimension
/// \param[in] list               Nested initializer list
/// \return CovarianceMatrix instance
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
