#ifndef B0B1C2CE_5E51_4440_A2C1_E90EE2C82FB6
#define B0B1C2CE_5E51_4440_A2C1_E90EE2C82FB6

#include "motion/generic_predict_common.h"

#include "env/ego_motion.hpp" // IWYU pragma: keep

namespace tracking
{
namespace motion
{
namespace generic
{

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
inline void PredictCommon<MotionModel_, CovarianceMatrixPolicy_>::run(Storage&             data,
                                                                      const value_type     dt,
                                                                      const EgoMotionType& egoMotion)
{
  assert(dt >= static_cast<value_type>(0));
  auto& underlying = static_cast<MotionModel_&>(*this);

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

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // B0B1C2CE_5E51_4440_A2C1_E90EE2C82FB6
