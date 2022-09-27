#ifndef D33C0BB9_EF21_44C6_8DAD_0C38C418D824
#define D33C0BB9_EF21_44C6_8DAD_0C38C418D824

#include "base/atomic_types.h"
#include "motion/state_mem.h"
#include "env/ego_motion.h"
#include "filter/kalman_filter.h"

namespace tracking
{
namespace motion
{

template <typename FloatType>
class IMotionModel
{
public:
  IMotionModel()                                   = default;
  IMotionModel(const IMotionModel<FloatType>&)     = default;
  IMotionModel(IMotionModel<FloatType>&&) noexcept = default;
  auto operator=(const IMotionModel<FloatType>&) -> IMotionModel<FloatType>& = default;
  auto operator=(IMotionModel<FloatType>&&) noexcept -> IMotionModel<FloatType>& = default;
  virtual ~IMotionModel()                                                        = default;

  virtual auto getX() const -> FloatType  = 0;
  virtual auto getVx() const -> FloatType = 0;
  
  // prediction is a functionality of the MotionModel (filter shall not know about egomotion compensation ...), tbd
  virtual void predict(const FloatType                        dt,
                       const filter::KalmanFilter<FloatType>& filter,
                       const env::EgoMotion<FloatType>&       egoMotion) = 0;
};

template <typename MotionModel,
          template <typename FloatType, sint32 Size>
          class CovarianceMatrixType,
          typename FloatType,
          sint32 Size>
class ExtendedMotionModel
    : public IMotionModel<FloatType>
    , public StateMem<CovarianceMatrixType, FloatType, Size>
{
public:
  using typename StateMem<CovarianceMatrixType, FloatType, Size>::StateVec;
  using typename StateMem<CovarianceMatrixType, FloatType, Size>::StateCov;
  
  ExtendedMotionModel()                                                                                   = default;
  ExtendedMotionModel(const ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>&)     = default;
  ExtendedMotionModel(ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>&&) noexcept = default;
  auto operator=(const ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>&)
      -> ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>& = default;
  auto operator=(ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>&&) noexcept
      -> ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>& = default;
  virtual ~ExtendedMotionModel()                                                  = default;

  auto getX() const -> FloatType final { return this->getVec()[MotionModel::X]; }
};

} // namespace motion
} // namespace tracking

#endif // D33C0BB9_EF21_44C6_8DAD_0C38C418D824
