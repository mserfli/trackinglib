#ifndef C8D9E0F1_3A4B_4C5D_A6E7_8F9A0B1C2D3E
#define C8D9E0F1_3A4B_4C5D_A6E7_8F9A0B1C2D3E

#include "base/first_include.h"                       // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/matrix.hpp"                     // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"              // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp"          // IWYU pragma: keep
#include "math/linalg/vector.hpp"                     // IWYU pragma: keep

namespace tracking
{
namespace filter
{
namespace detail
{

/// \brief Decorrelate a measurement for sequential (scalar) processing
///
/// Sequential measurement updates process one measurement component at a time and therefore
/// require uncorrelated components. Using the UDU factorization R = U*D*U' the transformed
/// system
///     vec' = inv(U)*vec ;  H' = inv(U)*H
/// has the exactly diagonal measurement covariance R' = D, so the scalar updates on the
/// transformed system are equivalent to a block update with the correlated R.
///
/// - Already diagonal R: no transformation, R' = diag(R).
/// - Factored policy: U and D are stored directly — decorrelation needs no decomposition.
/// - Full policy with correlations: R.decomposeUDUT() provides U and D.
///
/// \tparam CovarianceMatrixPolicy_ Policy type that defines the covariance matrix implementation
/// \tparam DimX_ State dimension
/// \tparam DimZ_ Measurement dimension
/// \param[in,out] vec    Measurement-sized vector to transform (innovation or effective measurement)
/// \param[in,out] H      Measurement Jacobian to transform
/// \param[out]    rDiag  Diagonal of the decorrelated measurement covariance R' = D
/// \param[in]     R      Measurement covariance
/// \return true if decorrelation succeeded, false if the update must be skipped
///         (R could not be decomposed)
template <typename CovarianceMatrixPolicy_, sint32 DimX_, sint32 DimZ_>
[[nodiscard]] inline auto decorrelateMeasurement(
    math::Vector<typename CovarianceMatrixPolicy_::value_type, DimZ_>&           vec,
    math::Matrix<typename CovarianceMatrixPolicy_::value_type, DimZ_, DimX_>&    H,
    math::DiagonalMatrix<typename CovarianceMatrixPolicy_::value_type, DimZ_>&   rDiag,
    const typename CovarianceMatrixPolicy_::template Instantiate<DimZ_>&         R) -> bool
{
  using value_type = typename CovarianceMatrixPolicy_::value_type;

  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    // R is stored as U*D*U': the decorrelated covariance is D itself
    rDiag = R.D();

    bool hasCorrelations = false;
    for (sint32 row = 0; (row < DimZ_) && !hasCorrelations; ++row)
    {
      for (sint32 col = row + 1; col < DimZ_; ++col)
      {
        if (R.U().at_unsafe(row, col) != static_cast<value_type>(0.0))
        {
          hasCorrelations = true;
          break;
        }
      }
    }

    if (hasCorrelations)
    {
      vec = math::Vector<value_type, DimZ_>{R.U().solve(vec)};
      H   = R.U().solve(H);
    }
    return true;
  }
  else
  {
    bool hasCorrelations = false;
    for (sint32 row = 0; (row < DimZ_) && !hasCorrelations; ++row)
    {
      for (sint32 col = row + 1; col < DimZ_; ++col)
      {
        if (R.at_unsafe(row, col) != static_cast<value_type>(0.0))
        {
          hasCorrelations = true;
          break;
        }
      }
    }

    if (!hasCorrelations)
    {
      for (sint32 i = 0; i < DimZ_; ++i)
      {
        rDiag.at_unsafe(i) = R.at_unsafe(i, i);
      }
      return true;
    }

    const auto udu = R.decomposeUDUT();
    if (!udu.has_value())
    {
      return false; // defensive: R is not symmetric, the update must be skipped
    }
    rDiag = udu.value().second;
    vec   = math::Vector<value_type, DimZ_>{udu.value().first.solve(vec)};
    H     = udu.value().first.solve(H);
    return true;
  }
}

} // namespace detail
} // namespace filter
} // namespace tracking

#endif // C8D9E0F1_3A4B_4C5D_A6E7_8F9A0B1C2D3E
