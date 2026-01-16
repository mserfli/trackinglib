#ifndef D33C0BB9_EF21_44C6_8DAD_0C38C418D824
#define D33C0BB9_EF21_44C6_8DAD_0C38C418D824

#include "base/first_include.h" // IWYU pragma: keep
#include "env/ego_motion.h"
#include "filter/information_filter.h"
#include "filter/kalman_filter.h"
#include "math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/errors.h"
#include "motion/state_mem.h"
#include <type_traits>

namespace tracking
{
namespace motion
{
namespace generic
{

template <typename T, typename FloatType_, template <typename, sint32> class CovarianceMatrixType_>
class Predict; // Forward declaration only
} // namespace generic

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename FloatType>
class IMotionModel
{
public:
  // rule of 5 declarations
  IMotionModel()          = default;
  virtual ~IMotionModel() = default;

  virtual auto getX() const -> FloatType  = 0;
  virtual auto getVx() const -> FloatType = 0;
  virtual auto getAx() const -> FloatType = 0;
  virtual auto getY() const -> FloatType  = 0;
  virtual auto getVy() const -> FloatType = 0;
  virtual auto getAy() const -> FloatType = 0;

  /// \brief Predicts the underlying MotionModel with the given filter (includes ego motion compensation)
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  virtual void predict(const FloatType                        dt,
                       const filter::KalmanFilter<FloatType>& filter, // TODO(matthias): decide between overloading or base class
                       const env::EgoMotion<FloatType>&       egoMotion) = 0;

  virtual void predict(const FloatType                             dt,
                       const filter::InformationFilter<FloatType>& filter,
                       const env::EgoMotion<FloatType>&            egoMotion) = 0;

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround to keep following idententation
  // clang-format on

  // rule of 5 declarations (remaining declarations are protected according to A12-8-6)
  IMotionModel(const IMotionModel& other)                    = default;
  IMotionModel(IMotionModel&&) noexcept                      = default;
  auto operator=(const IMotionModel& other) -> IMotionModel& = default;
  auto operator=(IMotionModel&&) noexcept -> IMotionModel&   = default;
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
  using typename StateMem<CovarianceMatrixType, FloatType, Size>::StateVec;
  using typename StateMem<CovarianceMatrixType, FloatType, Size>::StateCov;

  // rule of 5 declarations
  ExtendedMotionModel()          = default;
  virtual ~ExtendedMotionModel() = default;

  /// \brief Create state vector from initializer list
  /// \param[in] list  Initializer list with state values
  /// \return StateVec
  static auto StateVecFromList(const std::initializer_list<FloatType>& list) -> StateVec
  {
    return tracking::math::Vector<FloatType, Size>::FromList(list);
  }

  /// \brief Create state covariance from initializer list
  /// \param[in] list  Nested initializer list with covariance values
  /// \return StateCov
  static auto StateCovFromList(const std::initializer_list<std::initializer_list<FloatType>>& list) -> StateCov
  {
    if constexpr (std::is_same_v<CovarianceMatrixType<FloatType, Size>, math::CovarianceMatrixFull<FloatType, Size>>)
    {
      return math::CovarianceMatrixFull<FloatType, Size>::FromList(list);
    }

    if constexpr (std::is_same_v<CovarianceMatrixType<FloatType, Size>, math::CovarianceMatrixFactored<FloatType, Size>>)
    {
      return tracking::math::conversions::CovarianceMatrixFactoredFromList<FloatType, Size>(list);
    }
  }

  /// \brief Create complete ExtendedMotionModel from initializer lists
  /// \param[in] vecList  Initializer list for state vector
  /// \param[in] covList  Nested initializer list for covariance matrix
  /// \return ExtendedMotionModel instance
  static auto FromLists(const std::initializer_list<FloatType>&                        vecList,
                        const std::initializer_list<std::initializer_list<FloatType>>& covList) -> MotionModel
  {
    auto vec = StateVecFromList(vecList);
    auto cov = StateCovFromList(covList);
    return MotionModel{vec, cov};
  }

  /// \brief Read access to x position
  /// \return FloatType
  auto getX() const -> FloatType final { return this->operator[](MotionModel::X); }

  /// \brief Read access to y position
  /// \return FloatType
  auto getY() const -> FloatType final { return this->operator[](MotionModel::Y); }

  /// \brief Inverts the state covariance matrix into information form and vice versa
  auto invertCov() -> tl::expected<void, math::Errors>;

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround to keep following idententation
  // clang-format on

  // rule of 5 declarations (remaining declarations are protected according to A12-8-6)
  ExtendedMotionModel(const ExtendedMotionModel& other)                    = default;
  ExtendedMotionModel(ExtendedMotionModel&&) noexcept                      = default;
  auto operator=(const ExtendedMotionModel& other) -> ExtendedMotionModel& = default;
  auto operator=(ExtendedMotionModel&&) noexcept -> ExtendedMotionModel&   = default;

  /// \brief Testing: Construct a new Extended Motion Model object
  /// \param[in] vec
  /// \param[in] cov
  explicit ExtendedMotionModel(const StateVec& vec, const StateCov& cov)
      : IMotionModel<FloatType>{}
      , StateMem<CovarianceMatrixType, FloatType, Size>{vec, cov}
  {
    assert(cov.determinant() > 0);
  }
};

template <typename MotionModel,
          template <typename FloatType, sint32 Size>
          class CovarianceMatrixType,
          typename FloatType,
          sint32 Size>
auto ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>::invertCov() -> tl::expected<void, math::Errors>
{
  auto&& res = this->getCov().inverse();
  if (res.has_value())
  {
    this->getCovForInternalUse() = res.value();
    return {};
  }
  else
  {
    return tl::unexpected<math::Errors>{res.error()};
  }
}

} // namespace motion
} // namespace tracking

#endif // D33C0BB9_EF21_44C6_8DAD_0C38C418D824
