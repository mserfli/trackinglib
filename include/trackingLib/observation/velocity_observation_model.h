#ifndef C3D4E5F6_8A9B_4C0D_B1E2_3F4A5B6C7D8E
#define C3D4E5F6_8A9B_4C0D_B1E2_3F4A5B6C7D8E

#include "base/first_include.h" // IWYU pragma: keep
#include "motion/state_def_traits.h"
#include "observation/extended_observation_model.h"
#include "observation/observation_model_traits.h"

namespace tracking
{
namespace observation
{

/// \brief Observation model for a direct velocity measurement (vx, vy)
///
/// Linear observation model measuring the velocity components of the state:
///     h(x) = [vx, vy]'
///
/// \note Measurements z and predictions h(x) are expressed in the sensor frame defined by the
///       mounting pose (see ExtendedObservationModel::getSensorPose()); an identity pose makes the
///       sensor frame coincide with the tracking frame.
///
/// \tparam CovarianceMatrixPolicy_ Policy type that defines the covariance matrix implementation
/// \tparam StateDef_ State definition structure of the observed motion model (requires VX and VY)
template <typename CovarianceMatrixPolicy_, typename StateDef_>
class VelocityObservationModel TEST_REMOVE_FINAL
    : public ExtendedObservationModel<VelocityObservationModel<CovarianceMatrixPolicy_, StateDef_>,
                                      ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, 2>>
{
public:
  static_assert(motion::has_velocity_v<StateDef_>, "VelocityObservationModel requires a StateDef providing VX and VY");

  enum MeasurementDef
  {
    MEAS_VX = 0,
    MEAS_VY,
    NUM_MEASUREMENT_VARIABLES
  };

  using instance_type                = VelocityObservationModel<CovarianceMatrixPolicy_, StateDef_>;
  using instance_trait               = ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, NUM_MEASUREMENT_VARIABLES>;
  using BaseExtendedObservationModel = ExtendedObservationModel<instance_type, instance_trait>;
  using value_type                   = typename instance_trait::value_type;
  using MeasurementVec               = typename BaseExtendedObservationModel::MeasurementVec;
  using MeasurementCov               = typename BaseExtendedObservationModel::MeasurementCov;
  using StateVec                     = typename BaseExtendedObservationModel::StateVec;
  using JacobianMatrix               = typename BaseExtendedObservationModel::JacobianMatrix;

  // rule of 5 declarations
  VelocityObservationModel()                                                       = default;
  VelocityObservationModel(const VelocityObservationModel&)                        = default;
  VelocityObservationModel(VelocityObservationModel&&) noexcept                    = default;
  auto operator=(const VelocityObservationModel&) -> VelocityObservationModel&     = default;
  auto operator=(VelocityObservationModel&&) noexcept -> VelocityObservationModel& = default;
  virtual ~VelocityObservationModel() TEST_REMOVE_FINAL                            = default;

  /// \brief Construct a new VelocityObservationModel with a validated covariance
  ///
  /// The release-safe, mandatory gate to the protected ctor below: validates cov before
  /// construction instead of relying on a debug-only precondition check. Defined here (not on
  /// the CRTP base) because only VelocityObservationModel's own members can reach its own
  /// protected ctor without a friend declaration, which AUTOSAR A11-3-1 prohibits.
  ///
  /// \param[in] vec  Measurement vector z = [vx, vy]'
  /// \param[in] cov  Measurement covariance R
  /// \return tl::expected containing the VelocityObservationModel instance on success, or
  ///         Errors::matrix_not_positive_definite if cov is not positive definite
  static auto TryCreate(const MeasurementVec& vec,
                        const MeasurementCov& cov) -> tl::expected<VelocityObservationModel, math::Errors>
  {
    if (!cov.isPositiveDefinite())
    {
      return tl::unexpected<math::Errors>{math::Errors::matrix_not_positive_definite};
    }
    return VelocityObservationModel{vec, cov};
  }

  /// \brief Construct a new VelocityObservationModel with a validated covariance and a sensor mounting pose
  ///
  /// The release-safe, mandatory gate to the protected ctor below: validates cov before
  /// construction instead of relying on a debug-only precondition check. Defined here (not on
  /// the CRTP base) because only VelocityObservationModel's own members can reach its own
  /// protected ctor without a friend declaration, which AUTOSAR A11-3-1 prohibits.
  ///
  /// \param[in] vec  Measurement vector z = [vx, vy]'
  /// \param[in] cov  Measurement covariance R
  /// \param[in] pose Static SE(2) sensor mounting pose relative to the tracking frame
  /// \return tl::expected containing the VelocityObservationModel instance on success, or
  ///         Errors::matrix_not_positive_definite if cov is not positive definite
  static auto TryCreate(const MeasurementVec&                                    vec,
                        const MeasurementCov&                                    cov,
                        const typename BaseExtendedObservationModel::SensorPose& pose)
      -> tl::expected<VelocityObservationModel, math::Errors>
  {
    if (!cov.isPositiveDefinite())
    {
      return tl::unexpected<math::Errors>{math::Errors::matrix_not_positive_definite};
    }
    return VelocityObservationModel{vec, cov, pose};
  }

  /// \brief Predict the measurement h(x) = [vx, vy]' for the given sensor-frame state
  ///
  /// The prediction is the target velocity relative to the sensor: the sensor's own lever-arm
  /// velocity (egoMotion.getVelocityAt() at the mounting position, rotated into the sensor frame)
  /// is subtracted from the target's sensor-frame velocity, mirroring
  /// RangeBearingDopplerObservationModel. Without this, a moving/turning sensor platform biases the
  /// predicted velocity by the full ego velocity at the mount.
  ///
  /// \param[in] state      Sensor-frame state vector the measurement is predicted for
  /// \param[in] egoMotion  Ego motion of the sensor platform
  /// \return MeasurementVec  Predicted measurement
  auto predictMeasurementSensorFrame(
      const StateVec& state, const typename BaseExtendedObservationModel::EgoMotionType& egoMotion) const -> MeasurementVec
  {
    const auto egoVelSensor = this->egoVelocitySensorFrame(egoMotion);

    MeasurementVec predicted{};
    predicted.at_unsafe(MEAS_VX) = state.at_unsafe(StateDef_::VX) - egoVelSensor.x();
    predicted.at_unsafe(MEAS_VY) = state.at_unsafe(StateDef_::VY) - egoVelSensor.y();
    return predicted;
  }

  /// \brief Compute the sensor-frame-local measurement Jacobian at the given sensor-frame state
  ///
  /// h(x) is affine in the sensor-frame velocity, so the local Jacobian is the identity; the base
  /// class chains the constant mounting rotation onto the tracking-frame state columns afterwards.
  ///
  /// \param[out] jacobian  The measurement Jacobian to be filled (sensor-frame-local partials)
  /// \param[in]  state     Sensor-frame state vector the Jacobian is linearized at (unused, model is linear)
  /// \param[in]  egoMotion Ego motion of the sensor platform (unused, model is a direct velocity measurement)
  void computeJacobianSensorFrame(JacobianMatrix&                                             jacobian,
                                  const StateVec&                                             state,
                                  const typename BaseExtendedObservationModel::EgoMotionType& egoMotion) const
  {
    static_cast<void>(state);
    static_cast<void>(egoMotion);
    jacobian.setZeros();
    jacobian.at_unsafe(MEAS_VX, StateDef_::VX) = static_cast<value_type>(1.0);
    jacobian.at_unsafe(MEAS_VY, StateDef_::VY) = static_cast<value_type>(1.0);
  }

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround to keep following idententation
  // clang-format on

  /// \brief Construct a new VelocityObservationModel given the measurement and its covariance
  ///
  /// Protected in production: use TryCreate() (validated) or FromLists() instead.
  ///
  /// \param[in] vec  Measurement vector z = [vx, vy]'
  /// \param[in] cov  Measurement covariance R
  explicit VelocityObservationModel(const MeasurementVec& vec, const MeasurementCov& cov)
      : BaseExtendedObservationModel{vec, cov}
  {
  }

  /// \brief Construct a new VelocityObservationModel given the measurement, its covariance and a sensor mounting pose
  ///
  /// Protected in production: use TryCreate() (validated) or FromLists() instead.
  ///
  /// \param[in] vec  Measurement vector z = [vx, vy]'
  /// \param[in] cov  Measurement covariance R
  /// \param[in] pose Static SE(2) sensor mounting pose relative to the tracking frame
  explicit VelocityObservationModel(const MeasurementVec&                                    vec,
                                    const MeasurementCov&                                    cov,
                                    const typename BaseExtendedObservationModel::SensorPose& pose)
      : BaseExtendedObservationModel{vec, cov, pose}
  {
  }
};

} // namespace observation
} // namespace tracking

#endif // C3D4E5F6_8A9B_4C0D_B1E2_3F4A5B6C7D8E
