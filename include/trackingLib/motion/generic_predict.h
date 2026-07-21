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
///
/// Mixed into the motion model via CRTP inheritance (see ExtendedMotionModel in imotion_model.h),
/// so a concrete motion model *is-a* Predict<Self, Policy>. This is required because
/// IMotionModel::predict() is a pure virtual method with a fixed (non-template) signature: only a
/// type that actually inherits Predict can override that virtual and forward to it. Because run()
/// is reached as an ordinary inherited instance method, it is non-static, and internally does
/// `static_cast<MotionModel_&>(*this)` to reach sibling state (e.g. getCovForInternalUse(), defined
/// on the StateMem base) that Predict itself does not have direct access to.
///
/// Contrast with generic::Update (generic_update.h): Update's run() is inherently a template
/// (variadic observation models, plus an UpdateMode_ tag), and C++ does not allow virtual template
/// member functions. With no virtual-interface pressure forcing an inheritance relationship,
/// Update is instead implemented as a stateless static utility, invoked by qualified name with the
/// motion model passed in explicitly rather than reached via CRTP self-cast. The static-vs-instance
/// difference between the two files is a consequence of this virtual-dispatch requirement, not an
/// inconsistency to reconcile.
///
/// \tparam MotionModel_             The underlying MotionModel
/// \tparam CovarianceMatrixPolicy_  Policy type that defines the covariance matrix implementation
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class Predict: public PredictCommon<MotionModel_, CovarianceMatrixPolicy_>
{
public:
  using value_type            = typename CovarianceMatrixPolicy_::value_type;
  using BasePredictCommon     = PredictCommon<MotionModel_, CovarianceMatrixPolicy_>;
  using KalmanFilterType      = filter::KalmanFilter<CovarianceMatrixPolicy_>;
  using InformationFilterType = filter::InformationFilter<CovarianceMatrixPolicy_>;
  using EgoMotionType         = env::EgoMotion<CovarianceMatrixPolicy_>;

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
