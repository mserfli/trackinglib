#ifndef D33C0BB9_EF21_44C6_8DAD_0C38C418D824
#define D33C0BB9_EF21_44C6_8DAD_0C38C418D824

#include "base/first_include.h"
#include "env/ego_motion.h"
#include "filter/kalman_filter.h"
#include "motion/state_mem.h"

namespace tracking
{
namespace motion
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename FloatType>
class IMotionModel
{
public:
  // rule of 5 declarations
  IMotionModel() = default;
  IMotionModel(const IMotionModel<FloatType>&) = default;
  IMotionModel(IMotionModel<FloatType>&&) noexcept = default;
  auto operator=(const IMotionModel<FloatType>&) -> IMotionModel<FloatType>& = default;
  auto operator=(IMotionModel<FloatType>&&) noexcept -> IMotionModel<FloatType>& = default;
  virtual ~IMotionModel() = default;

  virtual auto getX() const -> FloatType = 0;
  virtual auto getVx() const -> FloatType = 0;
  virtual auto getY() const -> FloatType = 0;
  virtual auto getVy() const -> FloatType = 0;

  /// \brief Predicts the underlying MotionModel with the given filter (includes ego motion compensation)
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  virtual void predict(const FloatType                        dt,
                       const filter::KalmanFilter<FloatType>& filter, // TODO(matthias): decide between overloading or base class
                       const env::EgoMotion<FloatType>&       egoMotion) = 0;
};

// clang-format off
template <typename MotionModel,
          template <typename FloatType, sint32 Size> class CovarianceMatrixType,
          typename FloatType,
          sint32 Size>
// clang-format on
class ExtendedMotionModel
    : public IMotionModel<FloatType>
    , public StateMem<CovarianceMatrixType, FloatType, Size>
{
public:
  using instance_type = ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>;
  using typename StateMem<CovarianceMatrixType, FloatType, Size>::StateVec;
  using typename StateMem<CovarianceMatrixType, FloatType, Size>::StateCov;

  // rule of 5 declarations
  ExtendedMotionModel() = default;
  ExtendedMotionModel(const instance_type&) = default;
  ExtendedMotionModel(instance_type&&) noexcept = default;
  auto operator=(const instance_type&) -> instance_type& = default;
  auto operator=(instance_type&&) noexcept -> instance_type& = default;
  virtual ~ExtendedMotionModel() = default;

  auto getX() const -> FloatType final { return this->getVec()[MotionModel::X]; }
  auto getY() const -> FloatType final { return this->getVec()[MotionModel::Y]; }
};

} // namespace motion
} // namespace tracking

#endif // D33C0BB9_EF21_44C6_8DAD_0C38C418D824
