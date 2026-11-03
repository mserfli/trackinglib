#include <gtest/gtest.h>
#include "base/atomic_types.h"
#include "math/linalg/covariance_matrix_policies.h"
#include "math/linalg/errors.h"
#include "motion/motion_model_ca.h"
#include "motion/motion_model_cv.h"
#include "trackingLib/motion/motion_model_ca.hpp" // IWYU pragma: keep -- Required template definitions.
#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep -- Required template definitions.
#include "trackingLib/observation/bearing_observation_model.h"
#include "trackingLib/observation/position_observation_model.h"
#include "trackingLib/observation/range_bearing_doppler_observation_model.h"
#include "trackingLib/observation/range_bearing_observation_model.h"
#include "trackingLib/observation/range_observation_model.h"
#include "trackingLib/observation/velocity_observation_model.h"
#include <initializer_list>
#include <utility>

namespace math        = tracking::math;
namespace motion      = tracking::motion;
namespace observation = tracking::observation;
using math::Errors;

template <typename Policy>
class CheckedModelFactories: public ::testing::Test
{
};
using ModelPolicies = ::testing::Types<math::FullCovarianceMatrixPolicy<float32>,
                                       math::FactoredCovarianceMatrixPolicy<float32>,
                                       math::FullCovarianceMatrixPolicy<float64>,
                                       math::FactoredCovarianceMatrixPolicy<float64>>;
TYPED_TEST_SUITE(CheckedModelFactories, ModelPolicies);

/// \brief Check all inherited motion helper contracts and their vector-first error order.
/// \tparam Model Built-in motion model
template <typename Model>
void checkMotionFactories()
{
  using T    = typename Model::value_type;
  using Flat = std::initializer_list<T>;
  using List = std::initializer_list<Flat>;
  static_assert(noexcept(Model::StateVecFromList(std::declval<const Flat&>())));
  static_assert(noexcept(Model::StateCovFromList(std::declval<const List&>())));
  static_assert(noexcept(Model::FromLists(std::declval<const Flat&>(), std::declval<const List&>())));
  static_assert(noexcept(
      Model::TryCreate(std::declval<const typename Model::StateVec&>(), std::declval<const typename Model::StateCov&>())));
  const auto bad = Model::FromLists({}, {});
  ASSERT_FALSE(bad.has_value());
  EXPECT_EQ(bad.error(), Errors::invalid_initializer_list_size);
  if constexpr (Model::CovarianceMatrixPolicy::is_factored)
  {
    static_assert(noexcept(Model::StateCovFromList(std::declval<const List&>(), std::declval<const Flat&>())));
    const auto badUD = Model::StateCovFromList(List{}, Flat{});
    ASSERT_FALSE(badUD.has_value());
    EXPECT_EQ(badUD.error(), Errors::invalid_initializer_list_row_count);
  }
}

/// \brief Check every observation factory overload and its inherited helpers.
/// \tparam Model Built-in observation model
template <typename Model>
// Assertion macro expansion inflates complexity; keep the factory contract checks together.
void checkObservationFactories() // NOLINT(readability-function-cognitive-complexity)
{
  using T    = typename Model::value_type;
  using Flat = std::initializer_list<T>;
  using List = std::initializer_list<Flat>;
  using Pose = typename Model::SensorPose;
  static_assert(noexcept(Model::MeasurementVecFromList(std::declval<const Flat&>())));
  static_assert(noexcept(Model::MeasurementCovFromList(std::declval<const List&>())));
  static_assert(noexcept(Model::FromLists(std::declval<const Flat&>(), std::declval<const List&>())));
  static_assert(
      noexcept(Model::FromLists(std::declval<const Flat&>(), std::declval<const List&>(), std::declval<const Pose&>())));
  static_assert(noexcept(Model::TryCreate(std::declval<const typename Model::MeasurementVec&>(),
                                          std::declval<const typename Model::MeasurementCov&>())));
  static_assert(noexcept(Model::TryCreate(std::declval<const typename Model::MeasurementVec&>(),
                                          std::declval<const typename Model::MeasurementCov&>(),
                                          std::declval<const Pose&>())));
  const auto bad     = Model::FromLists({}, {});
  const auto badPose = Model::FromLists({}, {}, Pose{});
  ASSERT_FALSE(bad.has_value());
  ASSERT_FALSE(badPose.has_value());
  EXPECT_EQ(bad.error(), Errors::invalid_initializer_list_size);
  EXPECT_EQ(badPose.error(), bad.error());
  if constexpr (Model::CovarianceMatrixPolicy::is_factored)
  {
    static_assert(noexcept(Model::MeasurementCovFromList(std::declval<const List&>(), std::declval<const Flat&>())));
    const auto badUD = Model::MeasurementCovFromList(List{}, Flat{});
    ASSERT_FALSE(badUD.has_value());
    EXPECT_EQ(badUD.error(), Errors::invalid_initializer_list_row_count);
  }
}

TYPED_TEST(CheckedModelFactories, FromLists__ChecksEveryBuiltInContract) // NOLINT
{
  using P = TypeParam;
  using S = motion::StateDefCV;
  checkMotionFactories<motion::MotionModelCV<P>>();
  checkMotionFactories<motion::MotionModelCA<P>>();
  checkObservationFactories<observation::PositionObservationModel<P, S>>();
  checkObservationFactories<observation::VelocityObservationModel<P, S>>();
  checkObservationFactories<observation::RangeObservationModel<P, S>>();
  checkObservationFactories<observation::BearingObservationModel<P, S>>();
  checkObservationFactories<observation::RangeBearingObservationModel<P, S>>();
  checkObservationFactories<observation::RangeBearingDopplerObservationModel<P, S>>();
}

TYPED_TEST(CheckedModelFactories, FromLists__PropagatesShapeSymmetryAndPolicySemantics) // NOLINT
{
  using P            = TypeParam;
  using T            = typename P::value_type;
  using CV           = motion::MotionModelCV<P>;
  using CA           = motion::MotionModelCA<P>;
  using Obs          = observation::PositionObservationModel<P, motion::StateDefCV>;
  const auto badCV   = CV::FromLists({0, 0, 0, 0}, {});
  const auto badCA   = CA::FromLists({0, 0, 0, 0, 0, 0}, {});
  const auto badObs  = Obs::FromLists({0, 0}, {});
  const auto badPose = Obs::FromLists({0, 0}, {}, typename Obs::SensorPose{});
  ASSERT_FALSE(badCV.has_value());
  ASSERT_FALSE(badCA.has_value());
  ASSERT_FALSE(badObs.has_value());
  ASSERT_FALSE(badPose.has_value());
  EXPECT_EQ(badCV.error(), Errors::invalid_initializer_list_row_count);
  EXPECT_EQ(badCA.error(), badCV.error());
  EXPECT_EQ(badObs.error(), badCV.error());
  EXPECT_EQ(badPose.error(), badCV.error());
  const auto asymmetric     = Obs::FromLists({0, 0}, {{1, 2}, {0, 1}});
  const auto asymmetricPose = Obs::FromLists({0, 0}, {{1, 2}, {0, 1}}, typename Obs::SensorPose{});
  ASSERT_FALSE(asymmetric.has_value());
  ASSERT_FALSE(asymmetricPose.has_value());
  EXPECT_EQ(asymmetric.error(), Errors::matrix_not_symmetric);
  EXPECT_EQ(asymmetricPose.error(), asymmetric.error());
  const auto valid   = Obs::FromLists({2, 3}, {{1, 0}, {0, 1}});
  const auto pose    = Obs::SensorPose::FromValues(T{4}, T{5}, T{0});
  const auto mounted = Obs::FromLists({2, 3}, {{1, 0}, {0, 1}}, pose);
  ASSERT_TRUE(valid.has_value());
  ASSERT_TRUE(mounted.has_value());
  EXPECT_EQ(mounted.value().getSensorPose().tx(), T{4});
  EXPECT_EQ(mounted.value().getSensorPose().ty(), T{5});
  const auto indefinite = CV::FromLists({0, 0, 0, 0}, {{-1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
  if constexpr (P::is_factored)
  {
    ASSERT_TRUE(indefinite.has_value());
    EXPECT_GT(indefinite.value().getCov().D().at_unsafe(0), T{0});
    const auto badLength   = Obs::MeasurementCovFromList({{2, 0}, {0, 1}}, {});
    const auto badUnit     = Obs::MeasurementCovFromList({{2, 0}, {0, 1}}, {0, -1});
    const auto badPositive = Obs::MeasurementCovFromList({{1, 0}, {99, 1}}, {0, -1});
    ASSERT_FALSE(badLength.has_value());
    ASSERT_FALSE(badUnit.has_value());
    ASSERT_FALSE(badPositive.has_value());
    EXPECT_EQ(badLength.error(), Errors::invalid_initializer_list_size);
    EXPECT_EQ(badUnit.error(), Errors::matrix_not_unit_upper_triangular);
    EXPECT_EQ(badPositive.error(), Errors::matrix_not_positive_definite);
  }
  else
  {
    ASSERT_FALSE(indefinite.has_value());
    EXPECT_EQ(indefinite.error(), Errors::matrix_not_positive_definite);
  }
}
