#ifndef C332F4D1_A269_4A00_B036_8FCB47171038
#define C332F4D1_A269_4A00_B036_8FCB47171038

#include "base/first_include.h" // IWYU pragma: keep
#include "motion/state_def_traits.h"
#include "observation/extended_observation_model.h"
#include "observation/observation_model_traits.h"
#include <algorithm>
#include <cmath>

namespace tracking
{
namespace observation
{

/// \brief Observation model for a polar range-only measurement
///
/// Nonlinear observation model measuring the range of the position:
///     h(x) = sqrt(x^2 + y^2)
///
/// \tparam CovarianceMatrixPolicy_ Policy type that defines the covariance matrix implementation
/// \tparam StateDef_ State definition structure of the observed motion model (requires X and Y)
template <typename CovarianceMatrixPolicy_, typename StateDef_>
class RangeObservationModel TEST_REMOVE_FINAL
    : public ExtendedObservationModel<RangeObservationModel<CovarianceMatrixPolicy_, StateDef_>,
                                      ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, 1>>
{
public:
  static_assert(motion::has_position_v<StateDef_>, "RangeObservationModel requires a StateDef providing X and Y");

  enum MeasurementDef
  {
    MEAS_RANGE = 0,
    NUM_MEASUREMENT_VARIABLES
  };

  using instance_type                = RangeObservationModel<CovarianceMatrixPolicy_, StateDef_>;
  using instance_trait               = ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, NUM_MEASUREMENT_VARIABLES>;
  using BaseExtendedObservationModel = ExtendedObservationModel<instance_type, instance_trait>;
  using value_type                   = typename instance_trait::value_type;
  using MeasurementVec               = typename BaseExtendedObservationModel::MeasurementVec;
  using MeasurementCov               = typename BaseExtendedObservationModel::MeasurementCov;
  using StateVec                     = typename BaseExtendedObservationModel::StateVec;
  using JacobianMatrix               = typename BaseExtendedObservationModel::JacobianMatrix;

  /// \brief Lower bound of the squared range to protect the Jacobian against division by zero
  static constexpr value_type RANGE_SQ_MIN = static_cast<value_type>(1e-6);

  // rule of 5 declarations
  RangeObservationModel()                                                    = default;
  RangeObservationModel(const RangeObservationModel&)                        = default;
  RangeObservationModel(RangeObservationModel&&) noexcept                    = default;
  auto operator=(const RangeObservationModel&) -> RangeObservationModel&     = default;
  auto operator=(RangeObservationModel&&) noexcept -> RangeObservationModel& = default;
  virtual ~RangeObservationModel() TEST_REMOVE_FINAL                         = default;

  /// \brief Construct a new RangeObservationModel given the measurement and its covariance
  /// \param[in] vec  Measurement vector z = [range]'
  /// \param[in] cov  Measurement covariance R
  explicit RangeObservationModel(const MeasurementVec& vec, const MeasurementCov& cov)
      : BaseExtendedObservationModel{vec, cov}
  {
  }

  /// \brief Predict the measurement h(x) = sqrt(x^2+y^2) for the given state
  /// \param[in] state  State vector the measurement is predicted for
  /// \return MeasurementVec  Predicted measurement
  auto predictMeasurement(const StateVec& state) const -> MeasurementVec
  {
    const value_type x = state.at_unsafe(StateDef_::X);
    const value_type y = state.at_unsafe(StateDef_::Y);

    MeasurementVec predicted{};
    predicted.at_unsafe(MEAS_RANGE) = std::sqrt((x * x) + (y * y));
    return predicted;
  }

  /// \brief Compute the measurement Jacobian H = dh/dx at the given state
  /// \param[out] jacobian  The measurement Jacobian to be filled
  /// \param[in]  state     State vector the Jacobian is linearized at
  /// \note The squared range is clamped to RANGE_SQ_MIN to protect against division by zero
  void computeJacobian(JacobianMatrix& jacobian, const StateVec& state) const
  {
    const value_type x       = state.at_unsafe(StateDef_::X);
    const value_type y       = state.at_unsafe(StateDef_::Y);
    const value_type rangeSq = std::max((x * x) + (y * y), RANGE_SQ_MIN);
    const value_type range   = std::sqrt(rangeSq);

    jacobian.setZeros();
    jacobian.at_unsafe(MEAS_RANGE, StateDef_::X) = x / range;
    jacobian.at_unsafe(MEAS_RANGE, StateDef_::Y) = y / range;
  }
};

} // namespace observation
} // namespace tracking

#endif // C332F4D1_A269_4A00_B036_8FCB47171038
