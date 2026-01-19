#ifndef A9EE36F9_16A8_4666_9CFE_801E34978BAC
#define A9EE36F9_16A8_4666_9CFE_801E34978BAC

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_policies.h"


namespace tracking
{
namespace math
{
namespace contract
{

template <typename PolicyType>
struct CovarianceMatrixPolicyIntf
{
  CovarianceMatrixPolicyIntf()
  {
    // Check inheritance from base policy
    static_assert(std::is_base_of_v<math::CovarianceMatrixPolicyBase, PolicyType>,
                  "CovarianceMatrixPolicy must inherit from CovarianceMatrixPolicyBase");

    // Check that it has the required FloatType alias
    static_assert(requires { PolicyType::is_factored; }, "CovarianceMatrixPolicy must define is_factored flag");

    // Check that it has the required FloatType alias
    static_assert(requires { typename PolicyType::FloatType; }, "CovarianceMatrixPolicy must define FloatType");

    // Check that it has the required Instantiate template
    static_assert(
        requires { typename PolicyType::template Instantiate<4>; }, "CovarianceMatrixPolicy must define Instantiate template");

    // Additional check to ensure it's one of the concrete policy types
    static_assert(std::is_same_v<PolicyType, math::FullCovarianceMatrixPolicy<typename PolicyType::FloatType>> ||
                      std::is_same_v<PolicyType, math::FactoredCovarianceMatrixPolicy<typename PolicyType::FloatType>>,
                  "CovarianceMatrixPolicy must be either FullCovarianceMatrixPolicy or FactoredCovarianceMatrixPolicy");
  }
};

} // namespace contract
} // namespace math
} // namespace tracking


#endif // A9EE36F9_16A8_4666_9CFE_801E34978BAC
