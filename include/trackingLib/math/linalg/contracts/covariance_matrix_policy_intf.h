#ifndef A9EE36F9_16A8_4666_9CFE_801E34978BAC
#define A9EE36F9_16A8_4666_9CFE_801E34978BAC

#include "base/first_include.h"                     // IWYU pragma: keep
#include "math/linalg/covariance_matrix_policies.h" // IWYU pragma: keep


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
#if __cplusplus == 202002L
    // Check inheritance from base policy
    static_assert(std::is_base_of_v<math::CovarianceMatrixPolicyBase, PolicyType>,
                  "CovarianceMatrixPolicy must inherit from CovarianceMatrixPolicyBase");

    // Check that it has the required value_type alias
    static_assert(requires { PolicyType::is_factored; }, "CovarianceMatrixPolicy must define is_factored flag");

    // Check that it has the required value_type alias
    static_assert(requires { typename PolicyType::value_type; }, "CovarianceMatrixPolicy must define value_type");

    // Check that it has the required Instantiate template
    static_assert(
        requires { typename PolicyType::template Instantiate<4>; }, "CovarianceMatrixPolicy must define Instantiate template");

    // Additional check to ensure it's one of the concrete policy types
    static_assert(std::is_same_v<PolicyType, math::FullCovarianceMatrixPolicy<typename PolicyType::value_type>> ||
                      std::is_same_v<PolicyType, math::FactoredCovarianceMatrixPolicy<typename PolicyType::value_type>>,
                  "CovarianceMatrixPolicy must be either FullCovarianceMatrixPolicy or FactoredCovarianceMatrixPolicy");
#endif //__cplusplus == 202002L
  }
};

} // namespace contract
} // namespace math
} // namespace tracking


#endif // A9EE36F9_16A8_4666_9CFE_801E34978BAC
