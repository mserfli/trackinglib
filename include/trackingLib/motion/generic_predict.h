#ifndef CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
#define CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5

#include "base/first_include.h" // IWYU pragma: keep
#include "env/ego_motion.h"
#include "filter/information_filter.h"
#include "filter/kalman_filter.h"
#include "motion/generic_predict_common.h"

namespace tracking
{
namespace motion
{
namespace generic
{

/// \brief Base class to extend any motion model with a generic prediction functionality using the CRTP pattern
/// \tparam MotionModel_             The underlying MotionModel
/// \tparam CovarianceMatrixPolicy_  Policy type that defines the covariance matrix implementation
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class Predict: public PredictCommon<MotionModel_, CovarianceMatrixPolicy_>
{
public:
  using super_predict_common_type = PredictCommon<MotionModel_, CovarianceMatrixPolicy_>;
  using value_type                = typename CovarianceMatrixPolicy_::value_type;
  using KalmanFilterType          = filter::KalmanFilter<CovarianceMatrixPolicy_>;
  using InformationFilterType     = filter::InformationFilter<CovarianceMatrixPolicy_>;
  using EgoMotionType             = env::EgoMotion<CovarianceMatrixPolicy_>;

  /// \brief State prediction with ego motion compensation using a KalmanFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const value_type dt, const KalmanFilterType& filter, const EgoMotionType& egoMotion);

  /// \brief State prediction with ego motion compensation using an InformationFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const value_type dt, const InformationFilterType& filter, const EgoMotionType& egoMotion);
};

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
