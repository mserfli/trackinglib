#ifndef B7C1D2E3_4F5A_4B6C_8D9E_0F1A2B3C4D5E
#define B7C1D2E3_4F5A_4B6C_8D9E_0F1A2B3C4D5E

#include "base/first_include.h" // IWYU pragma: keep
#include <type_traits>

namespace tracking
{
namespace filter
{
namespace update_mode
{

/// \brief Tag type selecting the block measurement update
///
/// All measurement components are applied in a single DimZ-sized update. This is the
/// numerically preferred mode for the full covariance representation and supports
/// arbitrary (also correlated) measurement covariance matrixes R.
struct Block
{
};

/// \brief Tag type selecting the sequential measurement update
///
/// The measurement components are applied one by one as scalar updates (rank-1 covariance
/// modifications). This is the numerically preferred mode for the UDU-factored covariance
/// representation as it keeps the factorization intact.
///
/// \note A correlated measurement covariance R is supported: the measurement system is
///       decorrelated first via R = U*D*U' (see filter::detail::decorrelateMeasurement),
///       which is a no-op for an already diagonal R.
struct Sequential
{
};

/// \brief Default update mode for the given covariance matrix policy
///
/// Selects the numerically appropriate algorithm for the covariance representation:
/// - full covariance -> Block update
/// - UDU-factored covariance -> Sequential update (keeps the factorization intact)
///
/// \tparam CovarianceMatrixPolicy_ Policy type that defines the covariance matrix implementation
template <typename CovarianceMatrixPolicy_>
using Default = std::conditional_t<CovarianceMatrixPolicy_::is_factored, Sequential, Block>;

} // namespace update_mode
} // namespace filter
} // namespace tracking

#endif // B7C1D2E3_4F5A_4B6C_8D9E_0F1A2B3C4D5E
