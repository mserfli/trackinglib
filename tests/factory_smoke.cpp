#include "trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "trackingLib/motion/motion_model_ca.hpp"
#include "trackingLib/motion/motion_model_cv.hpp"
#include "trackingLib/observation/bearing_observation_model.h"
#include "trackingLib/observation/position_observation_model.h"
#include "trackingLib/observation/range_bearing_doppler_observation_model.h"
#include "trackingLib/observation/range_bearing_observation_model.h"
#include "trackingLib/observation/range_observation_model.h"
#include "trackingLib/observation/velocity_observation_model.h"

#include <initializer_list>
#include <limits>
#include <type_traits>
#include <utility>

#ifdef TEST_BUILD
#error Factory smoke must exercise ordinary public access
#endif

namespace
{
using namespace tracking;
using math::Errors;

static_assert(std::is_same<std::underlying_type_t<Errors>, int>::value, "Preserve error enum representation");
static_assert(static_cast<int>(Errors::invalid_access_row) == 0);
static_assert(static_cast<int>(Errors::invalid_access_col) == 1);
static_assert(static_cast<int>(Errors::invalid_access_idx) == 2);
static_assert(static_cast<int>(Errors::divide_by_zero) == 3);
static_assert(static_cast<int>(Errors::not_integer_division) == 4);
static_assert(static_cast<int>(Errors::integer_divide_overflows) == 5);
static_assert(static_cast<int>(Errors::matrix_not_symmetric) == 6);
static_assert(static_cast<int>(Errors::matrix_not_positive_definite) == 7);

/// \brief Test an error without accessing an unsuccessful value.
/// \tparam T Factory value type
/// \param[in] result Checked factory result
/// \param[in] error Expected error
/// \return Whether the result reports the expected error
template <typename T>
auto hasError(const tl::expected<T, Errors>& result, Errors error) -> bool
{
  return !result.has_value() && result.error() == error;
}

/// \brief Instantiate primitive factories and exercise malformed lists without assertions.
/// \tparam T Primitive element type
/// \return Whether all primitive checks pass
template <typename T>
auto primitives() -> bool
{
  using Flat     = std::initializer_list<T>;
  using List     = std::initializer_list<Flat>;
  using Row      = math::Matrix<T, 2, 2, true>;
  using Col      = math::Matrix<T, 2, 2, false>;
  using Square   = math::SquareMatrix<T, 2>;
  using Vector   = math::Vector<T, 2>;
  using Diagonal = math::DiagonalMatrix<T, 2>;
  using Lower    = math::TriangularMatrix<T, 2, true, false>;
  using Upper    = math::TriangularMatrix<T, 2, false, true>;
  static_assert(noexcept(Row::FromList(std::declval<const List&>())));
  static_assert(noexcept(Col::FromList(std::declval<const List&>())));
  static_assert(noexcept(Square::FromList(std::declval<const List&>())));
  static_assert(noexcept(Vector::FromList(std::declval<const Flat&>())));
  static_assert(noexcept(Diagonal::FromList(std::declval<const Flat&>())));
  static_assert(noexcept(Diagonal::FromList(std::declval<const List&>())));
  static_assert(noexcept(Lower::FromList(std::declval<const List&>())));
  static_assert(noexcept(Upper::FromList(std::declval<const List&>())));
  const List list{{1, 2}, {3, 4}};
  const auto row      = Row::FromList(list);
  const auto col      = Col::FromList(list);
  const auto square   = Square::FromList(list);
  const auto vector   = Vector::FromList({1, 4});
  const auto diagonal = Diagonal::FromList(list);
  const auto flat     = Diagonal::FromList(Flat{1, 4});
  const auto lower    = Lower::FromList(list);
  const auto upper    = Upper::FromList(list);
  if (!row || !col || !square || !vector || !diagonal || !flat || !lower || !upper)
  {
    return false;
  }
  if (row.value().at_unsafe(1, 0) != T{3} || col.value().at_unsafe(0, 1) != T{2} || square.value().at_unsafe(1, 1) != T{4} ||
      vector.value().at_unsafe(1) != T{4} || diagonal.value().at_unsafe(1) != T{4} || flat.value().at_unsafe(0) != T{1} ||
      lower.value().at_unsafe(1, 0) != T{3} || upper.value().at_unsafe(0, 1) != T{2})
  {
    return false;
  }
  for (const auto& bad : {List{}, List{{1, 2}}, List{{1, 2}, {3, 4}, {5, 6}}})
  {
    if (!hasError(Row::FromList(bad), Errors::invalid_initializer_list_row_count) ||
        !hasError(Col::FromList(bad), Errors::invalid_initializer_list_row_count) ||
        !hasError(Square::FromList(bad), Errors::invalid_initializer_list_row_count) ||
        !hasError(Lower::FromList(bad), Errors::invalid_initializer_list_row_count) ||
        !hasError(Diagonal::FromList(bad), Errors::invalid_initializer_list_row_count))
    {
      return false;
    }
  }
  for (const auto& bad : {List{{}, {}}, List{{1}, {2}}, List{{1, 2}, {3}}, List{{1, 2}, {3, 4, 5}}})
  {
    if (!hasError(Row::FromList(bad), Errors::invalid_initializer_list_col_count) ||
        !hasError(Col::FromList(bad), Errors::invalid_initializer_list_col_count) ||
        !hasError(Square::FromList(bad), Errors::invalid_initializer_list_col_count) ||
        !hasError(Upper::FromList(bad), Errors::invalid_initializer_list_col_count) ||
        !hasError(Diagonal::FromList(bad), Errors::invalid_initializer_list_col_count))
    {
      return false;
    }
  }
  for (const auto& bad : {Flat{}, Flat{1}, Flat{1, 2, 3}})
  {
    if (!hasError(Vector::FromList(bad), Errors::invalid_initializer_list_size) ||
        !hasError(Diagonal::FromList(bad), Errors::invalid_initializer_list_size))
    {
      return false;
    }
  }
  return true;
}

/// \brief Verify covariance boundaries and retained full-list UDU clamping.
/// \tparam T Floating-point element type
/// \return Whether all covariance checks pass
template <typename T>
auto covariances() -> bool
{
  using Full     = math::CovarianceMatrixFull<T, 2>;
  using Factored = math::CovarianceMatrixFactored<T, 2>;
  using Flat     = std::initializer_list<T>;
  using List     = std::initializer_list<Flat>;
  static_assert(noexcept(Full::FromList(std::declval<const List&>())));
  static_assert(noexcept(Factored::FromList(std::declval<const List&>(), std::declval<const Flat&>())));
  static_assert(noexcept(math::conversions::CovarianceMatrixFactoredFromList<T, 2>(std::declval<const List&>())));
  const auto full       = Full::FromList({{-1, 0}, {0, 1}});
  const auto clamped    = math::conversions::CovarianceMatrixFactoredFromList<T, 2>({{-1, 0}, {0, 1}});
  const auto singular   = Full::FromList({{0, 0}, {0, 0}});
  const auto explicitUD = Factored::FromList({{1, 2}, {99, 1}}, {3, 4});
  if (!full || !clamped || !singular || !explicitUD)
  {
    return false;
  }
  if (full.value().isPositiveDefinite() || !clamped.value().isPositiveDefinite() ||
      clamped.value().D().at_unsafe(0) != std::numeric_limits<T>::epsilon() || explicitUD.value().U().at_unsafe(0, 1) != T{2})
  {
    return false;
  }
  return hasError(Full::FromList({{1, 2}, {0, 1}}), Errors::matrix_not_symmetric) &&
         hasError(math::conversions::CovarianceMatrixFactoredFromList<T, 2>({{1, 2}, {0, 1}}), Errors::matrix_not_symmetric) &&
         hasError(Factored::FromList({}, {}), Errors::invalid_initializer_list_row_count) &&
         hasError(Factored::FromList({{1, 0}, {0}}, {}), Errors::invalid_initializer_list_col_count) &&
         hasError(Factored::FromList({{2, 0}, {0, 1}}, {}), Errors::invalid_initializer_list_size) &&
         hasError(Factored::FromList({{2, 0}, {0, 1}}, {0, -1}), Errors::matrix_not_unit_upper_triangular) &&
         hasError(Factored::FromList({{1, 0}, {99, 1}}, {0, 1}), Errors::matrix_not_positive_definite) &&
         hasError(Factored::FromList({{1, 0}, {99, 1}}, {1, -1}), Errors::matrix_not_positive_definite);
}

/// \brief Instantiate motion helpers and complete checked CV/CA construction.
/// \tparam Model Concrete motion model
/// \return Whether model checks pass
template <typename Model>
auto motionModel() -> bool
{
  using T    = typename Model::value_type;
  using Flat = std::initializer_list<T>;
  using List = std::initializer_list<Flat>;
  static_assert(noexcept(Model::StateVecFromList(std::declval<const Flat&>())));
  static_assert(noexcept(Model::StateCovFromList(std::declval<const List&>())));
  static_assert(noexcept(Model::FromLists(std::declval<const Flat&>(), std::declval<const List&>())));
  static_assert(noexcept(
      Model::TryCreate(std::declval<const typename Model::StateVec&>(), std::declval<const typename Model::StateCov&>())));
  if (!hasError(Model::StateVecFromList({}), Errors::invalid_initializer_list_size) ||
      !hasError(Model::StateCovFromList({}), Errors::invalid_initializer_list_row_count) ||
      !hasError(Model::FromLists({}, {}), Errors::invalid_initializer_list_size))
  {
    return false;
  }
  if constexpr (Model::CovarianceMatrixPolicy::is_factored)
  {
    static_assert(noexcept(Model::StateCovFromList(std::declval<const List&>(), std::declval<const Flat&>())));
    if (!hasError(Model::StateCovFromList({}, {}), Errors::invalid_initializer_list_row_count))
    {
      return false;
    }
  }
  if constexpr (Model::StateVec::Rows == 4)
  {
    const auto model      = Model::FromLists({2, 0, 3, 0}, {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    const auto indefinite = Model::FromLists({2, 0, 3, 0}, {{-1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    if (!model || model.value().getX() != T{2})
    {
      return false;
    }
    if constexpr (Model::CovarianceMatrixPolicy::is_factored)
    {
      return indefinite.has_value();
    }
    else
    {
      return hasError(indefinite, Errors::matrix_not_positive_definite);
    }
  }
  else
  {
    const auto model = Model::FromLists(
        {2, 0, 0, 3, 0, 0},
        {{1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}});
    return model.has_value() && model.value().getX() == T{2};
  }
}

/// \brief Instantiate observation helpers, including sensor-pose forwarding.
/// \tparam Model Concrete observation model
/// \return Whether observation checks pass
template <typename Model>
auto observationModel() -> bool
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
  if (!hasError(Model::MeasurementVecFromList({}), Errors::invalid_initializer_list_size) ||
      !hasError(Model::MeasurementCovFromList({}), Errors::invalid_initializer_list_row_count) ||
      !hasError(Model::FromLists({}, {}), Errors::invalid_initializer_list_size) ||
      !hasError(Model::FromLists({}, {}, Pose{}), Errors::invalid_initializer_list_size))
  {
    return false;
  }
  if constexpr (Model::CovarianceMatrixPolicy::is_factored)
  {
    static_assert(noexcept(Model::MeasurementCovFromList(std::declval<const List&>(), std::declval<const Flat&>())));
    if (!hasError(Model::MeasurementCovFromList({}, {}), Errors::invalid_initializer_list_row_count))
    {
      return false;
    }
  }
  const auto pose = Pose::FromValues(T{4}, T{5}, T{0});
  if constexpr (Model::MeasurementVec::Rows == 1)
  {
    const auto result = Model::FromLists({2}, {{1}}, pose);
    return result.has_value() && result.value().getSensorPose().tx() == T{4};
  }
  else if constexpr (Model::MeasurementVec::Rows == 2)
  {
    const auto result = Model::FromLists({2, 3}, {{1, 0}, {0, 1}}, pose);
    return result.has_value() && result.value().getSensorPose().tx() == T{4};
  }
  else
  {
    const auto result = Model::FromLists({2, 3, 4}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, pose);
    return result.has_value() && result.value().getSensorPose().tx() == T{4};
  }
}

/// \brief Exercise every built-in construction policy and model.
/// \tparam Policy Covariance policy
/// \return Whether all model checks pass
template <typename Policy>
auto models() -> bool
{
  using State = motion::StateDefCV;
  return motionModel<motion::MotionModelCV<Policy>>() && motionModel<motion::MotionModelCA<Policy>>() &&
         observationModel<observation::PositionObservationModel<Policy, State>>() &&
         observationModel<observation::VelocityObservationModel<Policy, State>>() &&
         observationModel<observation::RangeObservationModel<Policy, State>>() &&
         observationModel<observation::BearingObservationModel<Policy, State>>() &&
         observationModel<observation::RangeBearingObservationModel<Policy, State>>() &&
         observationModel<observation::RangeBearingDopplerObservationModel<Policy, State>>();
}
} // namespace

/// \brief Return a family-specific failure code; ordinary checks also run under NDEBUG.
/// \return Zero on success, or a factory-family failure code
int main()
{
  if (!primitives<sint32>() || !primitives<float32>() || !primitives<float64>())
  {
    return 1;
  }
  if (!covariances<float32>() || !covariances<float64>())
  {
    return 2;
  }
  if (!models<math::FullCovarianceMatrixPolicy<float32>>() || !models<math::FullCovarianceMatrixPolicy<float64>>())
  {
    return 3;
  }
  if (!models<math::FactoredCovarianceMatrixPolicy<float32>>() || !models<math::FactoredCovarianceMatrixPolicy<float64>>())
  {
    return 4;
  }
  return 0;
}
