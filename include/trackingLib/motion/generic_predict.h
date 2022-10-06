#ifndef CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
#define CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5

#include "base/first_include.h"
#include "env/ego_motion.h"
#include "filter/information_filter.h"
#include "filter/kalman_filter.h"
#include "motion/generic_predict_common.hpp"
#include "motion/state_mem.h"


namespace tracking
{
namespace motion
{
namespace generic
{

/// \brief Base class to extend any motion model with a generic prediction functionality using the CRTP pattern
/// \tparam MotionModel           The underlying MotionModel
/// \tparam FloatType             The float type representation
/// \tparam CovarianceMatrixType  The used covariance matrix type
template <typename MotionModel, typename FloatType, template <typename FloatType_, sint32 Size> class CovarianceMatrixType>
class Predict
{
};

/// \brief Partial specialization of the generic predictor for the basic square covariance matrix
/// \tparam MotionModel           The underlying MotionModel
/// \tparam FloatType             The float type representation
template <typename MotionModel, typename FloatType>
class Predict<MotionModel, FloatType, math::CovarianceMatrixFull>: public PredictCommon<MotionModel, FloatType>
{
public:
  /// \brief State prediction with ego motion compensation using a KalmanFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const FloatType dt, const filter::KalmanFilter<FloatType>& filter, const env::EgoMotion<FloatType>& egoMotion);

  /// \brief State prediction with ego motion compensation using an InformationFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const FloatType dt, const filter::InformationFilter<FloatType>& filter, const env::EgoMotion<FloatType>& egoMotion);
};

/// \brief Partial specialization of the generic predictor for a factored covariance matrix
/// \tparam MotionModel           The underlying MotionModel
/// \tparam FloatType             The float type representation
template <typename MotionModel, typename FloatType>
class Predict<MotionModel, FloatType, math::CovarianceMatrixFactored>: public PredictCommon<MotionModel, FloatType>
{
public:
  /// \brief State prediction with ego motion compensation using a KalmanFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const FloatType dt, const filter::KalmanFilter<FloatType>& filter, const env::EgoMotion<FloatType>& egoMotion);
};

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
