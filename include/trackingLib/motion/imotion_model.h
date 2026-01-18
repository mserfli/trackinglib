#ifndef D33C0BB9_EF21_44C6_8DAD_0C38C418D824
#define D33C0BB9_EF21_44C6_8DAD_0C38C418D824

#include "base/first_include.h" // IWYU pragma: keep
#include "env/ego_motion.h"
#include "filter/information_filter.h"
#include "filter/kalman_filter.h"
#include "math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "math/linalg/errors.h"
#include "motion/motion_model_traits.h" // IWYU pragma: keep
#include "motion/state_mem.h"

namespace tracking
{
namespace motion
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen

/// \brief Abstract Motion Model interface
/// \tparam CovarianceMatrixPolicy_  Policy type that defines the covariance matrix implementation
template <typename CovarianceMatrixPolicy_>
class IMotionModel
{
public:
  using FloatType             = typename CovarianceMatrixPolicy_::FloatType;
  using EgoMotionType         = env::EgoMotion<CovarianceMatrixPolicy_>;
  using KalmanFilterType      = filter::KalmanFilter<FloatType>;
  using InformationFilterType = filter::InformationFilter<FloatType>;

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
  virtual void predict(const FloatType dt, const KalmanFilterType& filter, const EgoMotionType& egoMotion) = 0;

  /// \brief Predicts the underlying MotionModel with the given filter (includes ego motion compensation)
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  virtual void predict(const FloatType dt, const InformationFilterType& filter, const EgoMotionType& egoMotion) = 0;

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

/// \brief Abstract MotionModel with known dimension, keeping the model memory for State and StateCovariance
/// \tparam MotionModel_
/// \tparam MotionModelTrait_
template <typename MotionModel_, typename MotionModelTrait_>
class ExtendedMotionModel
    : public IMotionModel<typename MotionModelTrait_::CovarianceMatrixPolicy>
    , public StateMem<typename MotionModelTrait_::CovarianceMatrixPolicy, MotionModelTrait_::Size>
{
public:
  using FloatType              = typename MotionModelTrait_::FloatType;
  using StateDef               = typename MotionModelTrait_::StateDef;
  using CovarianceMatrixPolicy = typename MotionModelTrait_::CovarianceMatrixPolicy;
  using BaseIMotionModel       = IMotionModel<CovarianceMatrixPolicy>;
  using BaseStateMem           = StateMem<CovarianceMatrixPolicy, MotionModelTrait_::Size>;
  using typename BaseStateMem::StateCov;
  using typename BaseStateMem::StateVec;

  // rule of 5 declarations
  ExtendedMotionModel()          = default;
  virtual ~ExtendedMotionModel() = default;

  /// \brief Create state vector from initializer list
  /// \param[in] list  Initializer list with state values
  /// \return StateVec
  static auto StateVecFromList(const std::initializer_list<FloatType>& list) -> StateVec { return StateVec::FromList(list); }

  /// \brief Create state covariance from initializer list
  /// \param[in] list  Nested initializer list with covariance values
  /// \return StateCov
  static auto StateCovFromList(const std::initializer_list<std::initializer_list<FloatType>>& list) -> StateCov
  {
    if constexpr (CovarianceMatrixPolicy::is_factored)
    {
      return math::conversions::CovarianceMatrixFactoredFromList<FloatType, MotionModelTrait_::Size>(list);
    }
    else
    {
      return StateCov::FromList(list);
    }
  }

  /// \brief Create factored state covariance from initializer list
  /// \param[in] u Nested initializer list for the upper triangular U matrix
  /// \param[in] d Flat initializer list for the diagonal D matrix
  /// \return StateCov
  template <typename T = CovarianceMatrixPolicy>
  static auto StateCovFromList(const std::initializer_list<std::initializer_list<FloatType>>& u,
                               const std::initializer_list<FloatType>& d) -> std::enable_if_t<T::is_factored, StateCov>
  {
    return StateCov::FromList(u, d);
  }

  /// \brief Create complete ExtendedMotionModel from initializer lists
  /// \param[in] vecList  Initializer list for state vector
  /// \param[in] covList  Nested initializer list for covariance matrix
  /// \return ExtendedMotionModel instance
  static auto FromLists(const std::initializer_list<FloatType>&                        vecList,
                        const std::initializer_list<std::initializer_list<FloatType>>& covList) -> MotionModel_
  {
    auto vec = StateVecFromList(vecList);
    auto cov = StateCovFromList(covList);
    return MotionModel_{vec, cov};
  }

  /// \brief Read access to x position
  /// \return FloatType
  auto getX() const -> FloatType final { return this->operator[](StateDef::X); }

  /// \brief Read access to y position
  /// \return FloatType
  auto getY() const -> FloatType final { return this->operator[](StateDef::Y); }

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
      : BaseIMotionModel{}
      , BaseStateMem{vec, cov}
  {
    assert(cov.determinant() > 0);
  }
};

template <typename MotionModel_, typename MotionModelTrait_>
auto ExtendedMotionModel<MotionModel_, MotionModelTrait_>::invertCov() -> tl::expected<void, math::Errors>
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
