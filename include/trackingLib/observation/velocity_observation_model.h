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

  /// \brief Construct a new VelocityObservationModel given the measurement and its covariance
  /// \param[in] vec  Measurement vector z = [vx, vy]'
  /// \param[in] cov  Measurement covariance R
  explicit VelocityObservationModel(const MeasurementVec& vec, const MeasurementCov& cov)
      : BaseExtendedObservationModel{vec, cov}
  {
  }

  /// \brief Construct a new VelocityObservationModel given the measurement, its covariance and a sensor mounting pose
  /// \param[in] vec  Measurement vector z = [vx, vy]'
  /// \param[in] cov  Measurement covariance R
  /// \param[in] pose Static SE(2) sensor mounting pose relative to the tracking frame
  explicit VelocityObservationModel(const MeasurementVec&                                    vec,
                                    const MeasurementCov&                                    cov,
                                    const typename BaseExtendedObservationModel::SensorPose& pose)
      : BaseExtendedObservationModel{vec, cov, pose}
  {
  }

  /// \brief Predict the measurement h(x) = [vx, vy]' for the given sensor-frame state
  /// \param[in] state  Sensor-frame state vector the measurement is predicted for
  /// \return MeasurementVec  Predicted measurement
  auto predictMeasurementSensorFrame(const StateVec& state) const -> MeasurementVec
  {
    MeasurementVec predicted{};
    predicted.at_unsafe(MEAS_VX) = state.at_unsafe(StateDef_::VX);
    predicted.at_unsafe(MEAS_VY) = state.at_unsafe(StateDef_::VY);
    return predicted;
  }

  /// \brief Compute the sensor-frame-local measurement Jacobian at the given sensor-frame state
  ///
  /// h(x) is affine in the sensor-frame velocity, so the local Jacobian is the identity; the base
  /// class chains the constant mounting rotation onto the tracking-frame state columns afterwards.
  ///
  /// \param[out] jacobian  The measurement Jacobian to be filled (sensor-frame-local partials)
  /// \param[in]  state     Sensor-frame state vector the Jacobian is linearized at (unused, model is linear)
  void computeJacobianSensorFrame(JacobianMatrix& jacobian, const StateVec& state) const
  {
    static_cast<void>(state);
    jacobian.setZeros();
    jacobian.at_unsafe(MEAS_VX, StateDef_::VX) = static_cast<value_type>(1.0);
    jacobian.at_unsafe(MEAS_VY, StateDef_::VY) = static_cast<value_type>(1.0);
  }
};

} // namespace observation
} // namespace tracking

#endif // C3D4E5F6_8A9B_4C0D_B1E2_3F4A5B6C7D8E
