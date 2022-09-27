#ifndef CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
#define CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5

#include "base/atomic_types.h"
#include "motion/state_mem.h"
#include "env/ego_motion.h"
#include "filter/kalman_filter.h"

namespace tracking
{
namespace motion
{
namespace generic
{

template <typename MotionModel>
struct PredictStorage
{
  typename MotionModel::StateMatrix               A{};
  typename MotionModel::EgoMotionMappingMatrix    Ge{};
  typename MotionModel::StateMatrix               Go{};
  typename MotionModel::ProcessNoiseDiagMatrix    Q{};
  typename MotionModel::ProcessNoiseMappingMatrix G{};
};

template <typename MotionModel, typename FloatType>
class PredictCommon
{
public:
  void run(PredictStorage<MotionModel>& data, const FloatType dt, const env::EgoMotion<FloatType>& egoMotion)
  {
    auto& underlying = static_cast<MotionModel&>(*this);

    // transform posteriori state into current frame
    underlying.compensateEgoMotion(data.Ge, data.Go, egoMotion);

    // apply state transition in current frame
    underlying.computeA(data.A, dt);
    underlying.applyDynamicalModel(dt);
    // post: state is predicted

    // calculate process noise and its contribution to the state
    underlying.computeQ(data.Q, dt);
    underlying.computeG(data.G, dt);
  }
};


template <typename MotionModel,
          typename FloatType,
          template <typename FloatType_, sint32 Size>
          class CovarianceMatrixType>
class Predict
{
};

template <typename MotionModel, typename FloatType>
class Predict<MotionModel, FloatType, math::CovarianceMatrixFull>: public PredictCommon<MotionModel, FloatType>
{
public:
  void run(const FloatType                        dt,
           const filter::KalmanFilter<FloatType>& filter,
           const env::EgoMotion<FloatType>&       egoMotion)
  {
    assert(dt >= 0.0F);
    auto& underlying = static_cast<MotionModel&>(*this);

    static PredictStorage<MotionModel> data;
    PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

    typename MotionModel::StateCovPtr P(
        new typename MotionModel::StateCov(data.Go * (underlying.getCov() * data.Go.transpose()) +
                                           data.Ge * (egoMotion.getDisplacementCog().cov * data.Ge.transpose())));

    filter.predictCovariance(*P, data.A, data.G, data.Q);

    underlying.setCov(std::move(P));
  }
};

template <typename MotionModel, typename FloatType>
class Predict<MotionModel, FloatType, math::CovarianceMatrixFactored>: public PredictCommon<MotionModel, FloatType>
{
public:
  void run(const FloatType                        dt,
           const filter::KalmanFilter<FloatType>& filter,
           const env::EgoMotion<FloatType>&       egoMotion)
  {
    assert(dt >= 0.0F);
    auto& underlying = static_cast<MotionModel&>(*this);

    static PredictStorage<MotionModel> data;
    PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

    static typename MotionModel::StateMatrix AGo = data.A * data.Go;

    static typename MotionModel::AugmentedProcessNoiseDiagMatrix    Qstar; // [De 0; 0 G]
    static typename MotionModel::AugmentedProcessNoiseMappingMatrix Gstar; // [A*Ge*Ue G]

    typename MotionModel::StateCovPtr P(new typename MotionModel::StateCov(underlying.getCov()));

    filter.predictCovariance(*P, AGo, Gstar, Qstar);

    underlying.setCov(std::move(P));
  }
};

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
