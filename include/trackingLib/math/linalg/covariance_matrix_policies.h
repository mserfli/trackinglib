#ifndef D891F0CE_1B63_4381_AA8F_2C4A973E5FF9
#define D891F0CE_1B63_4381_AA8F_2C4A973E5FF9

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"

namespace tracking
{
namespace math
{

struct CovarianceMatrixPolicyBase
{
};

template <typename FloatT>
struct FullCovarianceMatrixPolicy: CovarianceMatrixPolicyBase
{
  using FloatType = FloatT;

  template <sint32 Size>
  using Instantiate = CovarianceMatrixFull<FloatT, Size>;
};

template <typename FloatT>
struct FactoredCovarianceMatrixPolicy: CovarianceMatrixPolicyBase
{
  using FloatType = FloatT;

  template <sint32 Size>
  using Instantiate = CovarianceMatrixFactored<FloatT, Size>;
};

} // namespace math
} // namespace tracking

#endif // D891F0CE_1B63_4381_AA8F_2C4A973E5FF9
