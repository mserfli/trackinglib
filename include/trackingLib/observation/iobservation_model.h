#ifndef F0A1B2C3_5D6E_4F7A_8B9C_0D1E2F3A4B5C
#define F0A1B2C3_5D6E_4F7A_8B9C_0D1E2F3A4B5C

#include "base/first_include.h" // IWYU pragma: keep
#include "base/require_abstract_intf.h"
#include "math/linalg/contracts/covariance_matrix_policy_intf.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_policies.h"              // IWYU pragma: keep

namespace tracking
{
namespace observation
{

/// \brief Abstract Observation Model interface
///
/// Dimension-agnostic and non-templated base of the observation model hierarchy, mirroring
/// motion::IMotionModel. All dimension-dependent methods live on the ExtendedObservationModel
/// layer below; this interface only carries the policy contracts and the measurement dimension.
///
/// \tparam CovarianceMatrixPolicy_  Policy type that defines the covariance matrix implementation
template <typename CovarianceMatrixPolicy_>
class IObservationModel
    : public math::contract::CovarianceMatrixPolicyIntf<CovarianceMatrixPolicy_>
    , public base::contract::RequireAbstractIntf<IObservationModel<CovarianceMatrixPolicy_>>
{
public:
  using value_type = typename CovarianceMatrixPolicy_::value_type;

  // rule of 5 declarations
  IObservationModel()          = default;
  virtual ~IObservationModel() = default;

  /// \brief Read access to the measurement dimension
  /// \return sint32  Number of measurement components
  virtual auto getDim() const -> sint32 = 0;

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround to keep following idententation
  // clang-format on

  // rule of 5 declarations (remaining declarations are protected according to A12-8-6)
  IObservationModel(const IObservationModel& other)                    = default;
  IObservationModel(IObservationModel&&) noexcept                      = default;
  auto operator=(const IObservationModel& other) -> IObservationModel& = default;
  auto operator=(IObservationModel&&) noexcept -> IObservationModel&   = default;
};

} // namespace observation
} // namespace tracking

#endif // F0A1B2C3_5D6E_4F7A_8B9C_0D1E2F3A4B5C
