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

  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    data.AGo = typename MotionModel_::StateMatrix{data.A * data.Go};
    // clang-format off
    // fill a diagonal matrix representing the augmented process noise matrix Qstar = [De 0; 0 Q]
    data.Qstar.template setBlock<EgoMotionType::DS_NUM_VARIABLES, EgoMotionType::DS_NUM_VARIABLES, 0, 0>(
        egoMotion.getDisplacementCog().cov.D()); // set De starting at dest row 0
    data.Qstar.template setBlock<MotionModel_::NUM_PROC_NOISE_VARIABLES,
                                  MotionModel_::NUM_PROC_NOISE_VARIABLES,
                                  0,
                                  EgoMotionType::DS_NUM_VARIABLES>(
        data.Q); // set Q starting at dest row DS_NUM_VARIABLES

    // fill a normal matrix representing the augmented process noise mapping matrix Gstar = [A*Ge*Ue G]
    data.Gstar.template setBlock<MotionModel_::NUM_STATE_VARIABLES, EgoMotionType::DS_NUM_VARIABLES,
                                  MotionModel_::NUM_STATE_VARIABLES, EgoMotionType::DS_NUM_VARIABLES,
                                  0, 0, true,
                                  0, 0>(
        data.A * data.Ge * egoMotion.getDisplacementCog().cov.U()); // set A*Ge*Ue starting at dest (0,0)
    data.Gstar.template setBlock<MotionModel_::NUM_STATE_VARIABLES, MotionModel_::NUM_PROC_NOISE_VARIABLES,
                                  MotionModel_::NUM_STATE_VARIABLES, MotionModel_::NUM_PROC_NOISE_VARIABLES,
                                  0, 0, true,
                                  0, EgoMotionType::DS_NUM_VARIABLES>(
        data.G); // set G starting at dest (0, DS_NUM_VARIABLES)
    // clang-format on
  }
}

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // B0B1C2CE_5E51_4440_A2C1_E90EE2C82FB6
