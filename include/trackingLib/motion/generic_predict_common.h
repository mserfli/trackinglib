#ifndef DA3325A6_9F6B_4D55_87B2_336CB0BAF21E
#define DA3325A6_9F6B_4D55_87B2_336CB0BAF21E

#include "base/first_include.h"
#include "env/ego_motion.h"
#include "motion/motion_model_cv.h"

namespace tracking
{
namespace motion
{
namespace generic
{

/// \brief Base class for common calculations needed for any prediction
/// \tparam MotionModel  The underlying motion model
/// \tparam FloatType    The float type representation
template <typename MotionModel, typename FloatType>
class PredictCommon
{
public:
  /// \brief Structure to store the calculations
  struct Storage
  {
    /// \brief Go defines the transformation of the state caused by the ego motion
    typename MotionModel::StateMatrix Go{};
    /// \brief Ge defines the propagated errors of the ego motion to the state space
    typename MotionModel::EgoMotionMappingMatrix Ge{};

    /// \brief A defines the state transition from k to k+1 and is calculated as the Jacobian for nonlinear process models
    typename MotionModel::StateMatrix A{};
    /// \brief Q defines the process noise
    typename MotionModel::ProcessNoiseDiagMatrix Q{};
    /// \brief G defines the transformation of the process noise to the full state space
    typename MotionModel::ProcessNoiseMappingMatrix G{};
  };

  /// \brief Runner to calculate the common predict data for the predictor
  /// \param[out] data       Output data storage for all precomputed results
  /// \param[in]  dt         The delta time from last state to predicted state
  /// \param[in]  egoMotion  The known egoMotion from last state to predicted state
  void run(Storage& data, const FloatType dt, const env::EgoMotion<FloatType>& egoMotion)
  {
    assert(dt >= static_cast<FloatType>(0.0));
    auto& underlying = static_cast<MotionModel&>(*this);

    // transform posteriori state into current frame
    underlying.compensateEgoMotion(data.Ge, data.Go, egoMotion);

    // apply state transition in current frame
    underlying.computeA(data.A, dt);
    underlying.applyProcessModel(dt);
    // post: state is predicted

    // calculate process noise and its contribution to the state
    underlying.computeQ(data.Q, dt);
    underlying.computeG(data.G, dt);
  }
};

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // DA3325A6_9F6B_4D55_87B2_336CB0BAF21E
