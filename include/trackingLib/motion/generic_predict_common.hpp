#ifndef B0B1C2CE_5E51_4440_A2C1_E90EE2C82FB6
#define B0B1C2CE_5E51_4440_A2C1_E90EE2C82FB6

#include "motion/generic_predict_common.h"

namespace tracking
{
namespace motion
{
namespace generic
{

template <typename MotionModel, typename FloatType>
inline void PredictCommon<MotionModel, FloatType>::run(Storage& data, const FloatType dt, const env::EgoMotion<FloatType>& egoMotion)
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

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // B0B1C2CE_5E51_4440_A2C1_E90EE2C82FB6
