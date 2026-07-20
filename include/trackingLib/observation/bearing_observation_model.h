#ifndef BF21C42C_063C_44D7_836F_367BCA00B897
#define BF21C42C_063C_44D7_836F_367BCA00B897

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

/// \brief Observation model for a polar bearing-only measurement
///
/// Nonlinear observation model measuring the bearing of the position:
///     h(x) = atan2(y, x)
///
/// The bearing innovation is wrapped into [-pi, pi] by the shadowed computeInnovation().
///
/// \note Measurements z and predictions h(x) are expressed in the sensor frame defined by the
///       mounting pose (see ExtendedObservationModel::getSensorPose()); an identity pose makes the
///       sensor frame coincide with the tracking frame.
///
/// \tparam CovarianceMatrixPolicy_ Policy type that defines the covariance matrix implementation
/// \tparam StateDef_ State definition structure of the observed motion model (requires X and Y)
template <typename CovarianceMatrixPolicy_, typename StateDef_>
class BearingObservationModel TEST_REMOVE_FINAL
    : public ExtendedObservationModel<BearingObservationModel<CovarianceMatrixPolicy_, StateDef_>,
                                      ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, 1>>
{
public:
  static_assert(motion::has_position_v<StateDef_>, "BearingObservationModel requires a StateDef providing X and Y");

  enum MeasurementDef
  {
    MEAS_BEARING = 0,
    NUM_MEASUREMENT_VARIABLES
  };

  using instance_type                = BearingObservationModel<CovarianceMatrixPolicy_, StateDef_>;
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
  BearingObservationModel()                                                      = default;
  BearingObservationModel(const BearingObservationModel&)                        = default;
  BearingObservationModel(BearingObservationModel&&) noexcept                    = default;
  auto operator=(const BearingObservationModel&) -> BearingObservationModel&     = default;
  auto operator=(BearingObservationModel&&) noexcept -> BearingObservationModel& = default;
  virtual ~BearingObservationModel() TEST_REMOVE_FINAL                           = default;

  /// \brief Construct a new BearingObservationModel given the measurement and its covariance
  /// \param[in] vec  Measurement vector z = [bearing]'
  /// \param[in] cov  Measurement covariance R
  explicit BearingObservationModel(const MeasurementVec& vec, const MeasurementCov& cov)
      : BaseExtendedObservationModel{vec, cov}
  {
  }

  /// \brief Construct a new BearingObservationModel given the measurement, its covariance and a sensor mounting pose
  /// \param[in] vec  Measurement vector z = [bearing]'
  /// \param[in] cov  Measurement covariance R
  /// \param[in] pose Static SE(2) sensor mounting pose relative to the tracking frame
  explicit BearingObservationModel(const MeasurementVec&                                    vec,
                                   const MeasurementCov&                                    cov,
                                   const typename BaseExtendedObservationModel::SensorPose& pose)
      : BaseExtendedObservationModel{vec, cov, pose}
  {
  }

  /// \brief Predict the measurement h(x) = atan2(y,x) for the given sensor-frame state
  /// \param[in] state  Sensor-frame state vector the measurement is predicted for
  /// \return MeasurementVec  Predicted measurement
  auto predictMeasurementSensorFrame(const StateVec& state) const -> MeasurementVec
  {
    const value_type x = state.at_unsafe(StateDef_::X);
    const value_type y = state.at_unsafe(StateDef_::Y);

    MeasurementVec predicted{};
    predicted.at_unsafe(MEAS_BEARING) = std::atan2(y, x);
    return predicted;
  }

  /// \brief Compute the sensor-frame-local measurement Jacobian at the given sensor-frame state
  ///
  /// Fills the local partials [-y/rangeSq, x/rangeSq] w.r.t. the sensor-frame position; the base
  /// class chains the constant mounting rotation onto the tracking-frame state columns afterwards.
  ///
  /// \param[out] jacobian  The measurement Jacobian to be filled (sensor-frame-local partials)
  /// \param[in]  state     Sensor-frame state vector the Jacobian is linearized at
  /// \note The squared range is clamped to RANGE_SQ_MIN to protect against division by zero
  void computeJacobianSensorFrame(JacobianMatrix& jacobian, const StateVec& state) const
  {
    const value_type x       = state.at_unsafe(StateDef_::X);
    const value_type y       = state.at_unsafe(StateDef_::Y);
    const value_type rangeSq = std::max((x * x) + (y * y), RANGE_SQ_MIN);

    jacobian.setZeros();
    jacobian.at_unsafe(MEAS_BEARING, StateDef_::X) = -y / rangeSq;
    jacobian.at_unsafe(MEAS_BEARING, StateDef_::Y) = x / rangeSq;
  }

  /// \brief Compute the innovation nu = z - h(x) with the bearing wrapped into [-pi, pi]
  /// \param[in] measurement  Measurement vector z
  /// \param[in] predicted    Predicted measurement h(x)
  /// \return MeasurementVec  Innovation with wrapped bearing component
  auto computeInnovation(const MeasurementVec& measurement, const MeasurementVec& predicted) const -> MeasurementVec
  {
    MeasurementVec innovation{measurement - predicted};
    value_type&    bearing = innovation.at_unsafe(MEAS_BEARING);
    bearing                = std::atan2(std::sin(bearing), std::cos(bearing));
    return innovation;
  }
};

} // namespace observation
} // namespace tracking

#endif // BF21C42C_063C_44D7_836F_367BCA00B897
