#ifndef E5F6A7B8_0C1D_4E2F_D3A4_5B6C7D8E9F0A
#define E5F6A7B8_0C1D_4E2F_D3A4_5B6C7D8E9F0A

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

/// \brief Observation model for a polar range/bearing/doppler measurement (e.g. radar)
///
/// Nonlinear observation model measuring the polar coordinates of the position and the radial
/// (doppler) velocity:
///     h(x) = [sqrt(x^2 + y^2), atan2(y, x), (x*vxRel + y*vyRel)/sqrt(x^2 + y^2)]'
/// evaluated on the sensor-frame position/velocity (identity pose reduces to the tracking-frame
/// form shown above), where (vxRel, vyRel) is the target velocity relative to the sensor: the
/// sensor's own lever-arm velocity (EgoMotion::getVelocityAt() at the mounting position) is
/// subtracted before projecting onto the line of sight, so a moving sensor platform does not bias
/// the doppler measurement.
///
/// \note Measurements z and predictions h(x) are expressed in the sensor frame defined by the
///       mounting pose (see ExtendedObservationModel::getSensorPose()); an identity pose makes the
///       sensor frame coincide with the tracking frame.
///
/// The bearing innovation is wrapped into [-pi, pi] by the shadowed computeInnovation().
/// If the StateDef provides no velocity, the doppler component is predicted as zero with a zero
/// Jacobian row: the zero row yields a zero gain column, so the doppler measurement contributes
/// nothing to the state update (it is effectively ignored, it does NOT contribute via its
/// variance). With a correlated R the decorrelation may still mix the doppler component into the
/// range/bearing rows - that is correct conditioning on the correlated component, not a defect.
///
/// \tparam CovarianceMatrixPolicy_ Policy type that defines the covariance matrix implementation
/// \tparam StateDef_ State definition structure of the observed motion model (requires X and Y)
template <typename CovarianceMatrixPolicy_, typename StateDef_>
class RangeBearingDopplerObservationModel TEST_REMOVE_FINAL
    : public ExtendedObservationModel<RangeBearingDopplerObservationModel<CovarianceMatrixPolicy_, StateDef_>,
                                      ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, 3>>
{
public:
  static_assert(motion::has_position_v<StateDef_>, "RangeBearingDopplerObservationModel requires a StateDef providing X and Y");

  enum MeasurementDef
  {
    MEAS_RANGE = 0,
    MEAS_BEARING,
    MEAS_DOPPLER,
    NUM_MEASUREMENT_VARIABLES
  };

  using instance_type                = RangeBearingDopplerObservationModel<CovarianceMatrixPolicy_, StateDef_>;
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
  RangeBearingDopplerObservationModel()                                                                  = default;
  RangeBearingDopplerObservationModel(const RangeBearingDopplerObservationModel&)                        = default;
  RangeBearingDopplerObservationModel(RangeBearingDopplerObservationModel&&) noexcept                    = default;
  auto operator=(const RangeBearingDopplerObservationModel&) -> RangeBearingDopplerObservationModel&     = default;
  auto operator=(RangeBearingDopplerObservationModel&&) noexcept -> RangeBearingDopplerObservationModel& = default;
  virtual ~RangeBearingDopplerObservationModel() TEST_REMOVE_FINAL                                       = default;

  /// \brief Construct a new RangeBearingDopplerObservationModel given the measurement and its covariance
  /// \param[in] vec  Measurement vector z = [range, bearing, doppler]'
  /// \param[in] cov  Measurement covariance R
  explicit RangeBearingDopplerObservationModel(const MeasurementVec& vec, const MeasurementCov& cov)
      : BaseExtendedObservationModel{vec, cov}
  {
  }

  /// \brief Construct a new RangeBearingDopplerObservationModel given the measurement, its covariance and a sensor mounting pose
  /// \param[in] vec  Measurement vector z = [range, bearing, doppler]'
  /// \param[in] cov  Measurement covariance R
  /// \param[in] pose Static SE(2) sensor mounting pose relative to the tracking frame
  explicit RangeBearingDopplerObservationModel(const MeasurementVec&                                    vec,
                                               const MeasurementCov&                                    cov,
                                               const typename BaseExtendedObservationModel::SensorPose& pose)
      : BaseExtendedObservationModel{vec, cov, pose}
  {
  }

  /// \brief Predict the measurement h(x) = [range, bearing, doppler]' for the given sensor-frame state
  ///
  /// The doppler term uses the target velocity relative to the sensor: the sensor's own
  /// lever-arm velocity (egoMotion.getVelocityAt() at the mounting position, rotated into the
  /// sensor frame) is subtracted from the target's sensor-frame velocity before projecting onto
  /// the line of sight. Without this, a moving sensor platform biases every doppler measurement.
  ///
  /// \param[in] state      Sensor-frame state vector the measurement is predicted for
  /// \param[in] egoMotion  Ego motion of the sensor platform
  /// \return MeasurementVec  Predicted measurement
  auto predictMeasurementSensorFrame(
      const StateVec& state, const typename BaseExtendedObservationModel::EgoMotionType& egoMotion) const -> MeasurementVec
  {
    const value_type x     = state.at_unsafe(StateDef_::X);
    const value_type y     = state.at_unsafe(StateDef_::Y);
    const value_type range = std::sqrt(std::max((x * x) + (y * y), RANGE_SQ_MIN));

    MeasurementVec predicted{};
    predicted.at_unsafe(MEAS_RANGE)   = std::sqrt((x * x) + (y * y));
    predicted.at_unsafe(MEAS_BEARING) = std::atan2(y, x);
    if constexpr (motion::has_velocity_v<StateDef_>)
    {
      const value_type vx = state.at_unsafe(StateDef_::VX);
      const value_type vy = state.at_unsafe(StateDef_::VY);

      const auto       egoVelMount  = egoMotion.getVelocityAt(this->getSensorPose().tx(), this->getSensorPose().ty());
      const auto       egoVelSensor = this->getSensorPose().directionToSensorFrame(egoVelMount.x(), egoVelMount.y());
      const value_type vxRel        = vx - egoVelSensor.x();
      const value_type vyRel        = vy - egoVelSensor.y();

      predicted.at_unsafe(MEAS_DOPPLER) = ((x * vxRel) + (y * vyRel)) / range;
    }
    else
    {
      predicted.at_unsafe(MEAS_DOPPLER) = static_cast<value_type>(0.0);
    }
    return predicted;
  }

  /// \brief Compute the sensor-frame-local measurement Jacobian at the given sensor-frame state
  ///
  /// Fills all local partials (range/bearing/doppler) w.r.t. the sensor-frame position and
  /// velocity; the base class chains the constant mounting rotation onto the tracking-frame state
  /// columns afterwards, once per column pair (position, velocity).
  ///
  /// \param[out] jacobian  The measurement Jacobian to be filled (sensor-frame-local partials)
  /// \param[in]  state     Sensor-frame state vector the Jacobian is linearized at
  /// \param[in]  egoMotion Ego motion of the sensor platform (see predictMeasurementSensorFrame())
  /// \note The squared range is clamped to RANGE_SQ_MIN to protect against division by zero
  void computeJacobianSensorFrame(JacobianMatrix&                                             jacobian,
                                  const StateVec&                                             state,
                                  const typename BaseExtendedObservationModel::EgoMotionType& egoMotion) const
  {
    const value_type x       = state.at_unsafe(StateDef_::X);
    const value_type y       = state.at_unsafe(StateDef_::Y);
    const value_type rangeSq = std::max((x * x) + (y * y), RANGE_SQ_MIN);
    const value_type range   = std::sqrt(rangeSq);

    jacobian.setZeros();
    jacobian.at_unsafe(MEAS_RANGE, StateDef_::X)   = x / range;
    jacobian.at_unsafe(MEAS_RANGE, StateDef_::Y)   = y / range;
    jacobian.at_unsafe(MEAS_BEARING, StateDef_::X) = -y / rangeSq;
    jacobian.at_unsafe(MEAS_BEARING, StateDef_::Y) = x / rangeSq;
    if constexpr (motion::has_velocity_v<StateDef_>)
    {
      const value_type vx = state.at_unsafe(StateDef_::VX);
      const value_type vy = state.at_unsafe(StateDef_::VY);

      const auto       egoVelMount  = egoMotion.getVelocityAt(this->getSensorPose().tx(), this->getSensorPose().ty());
      const auto       egoVelSensor = this->getSensorPose().directionToSensorFrame(egoVelMount.x(), egoVelMount.y());
      const value_type vxRel        = vx - egoVelSensor.x();
      const value_type vyRel        = vy - egoVelSensor.y();
      const value_type radial       = (x * vxRel) + (y * vyRel);

      jacobian.at_unsafe(MEAS_DOPPLER, StateDef_::X)  = (vxRel / range) - ((x * radial) / (rangeSq * range));
      jacobian.at_unsafe(MEAS_DOPPLER, StateDef_::Y)  = (vyRel / range) - ((y * radial) / (rangeSq * range));
      jacobian.at_unsafe(MEAS_DOPPLER, StateDef_::VX) = x / range;
      jacobian.at_unsafe(MEAS_DOPPLER, StateDef_::VY) = y / range;
    }
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

#endif // E5F6A7B8_0C1D_4E2F_D3A4_5B6C7D8E9F0A
