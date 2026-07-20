#ifndef B2C3D4E5_7F8A_4B9C_A0D1_2E3F4A5B6C7D
#define B2C3D4E5_7F8A_4B9C_A0D1_2E3F4A5B6C7D

#include "base/first_include.h" // IWYU pragma: keep
#include "motion/state_def_traits.h"
#include "observation/extended_observation_model.h"
#include "observation/observation_model_traits.h"

namespace tracking
{
namespace observation
{

/// \brief Observation model for a direct position measurement (x, y)
///
/// Linear observation model measuring the position components of the state:
///     h(x) = [x, y]'
///
/// \note Measurements z and predictions h(x) are expressed in the sensor frame defined by the
///       mounting pose (see ExtendedObservationModel::getSensorPose()); an identity pose makes the
///       sensor frame coincide with the tracking frame.
///
/// \tparam CovarianceMatrixPolicy_ Policy type that defines the covariance matrix implementation
/// \tparam StateDef_ State definition structure of the observed motion model (requires X and Y)
template <typename CovarianceMatrixPolicy_, typename StateDef_>
class PositionObservationModel TEST_REMOVE_FINAL
    : public ExtendedObservationModel<PositionObservationModel<CovarianceMatrixPolicy_, StateDef_>,
                                      ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, 2>>
{
public:
  static_assert(motion::has_position_v<StateDef_>, "PositionObservationModel requires a StateDef providing X and Y");

  enum MeasurementDef
  {
    MEAS_X = 0,
    MEAS_Y,
    NUM_MEASUREMENT_VARIABLES
  };

  using instance_type                = PositionObservationModel<CovarianceMatrixPolicy_, StateDef_>;
  using instance_trait               = ObservationModelTraits<CovarianceMatrixPolicy_, StateDef_, NUM_MEASUREMENT_VARIABLES>;
  using BaseExtendedObservationModel = ExtendedObservationModel<instance_type, instance_trait>;
  using value_type                   = typename instance_trait::value_type;
  using MeasurementVec               = typename BaseExtendedObservationModel::MeasurementVec;
  using MeasurementCov               = typename BaseExtendedObservationModel::MeasurementCov;
  using StateVec                     = typename BaseExtendedObservationModel::StateVec;
  using JacobianMatrix               = typename BaseExtendedObservationModel::JacobianMatrix;

  // rule of 5 declarations
  PositionObservationModel()                                                       = default;
  PositionObservationModel(const PositionObservationModel&)                        = default;
  PositionObservationModel(PositionObservationModel&&) noexcept                    = default;
  auto operator=(const PositionObservationModel&) -> PositionObservationModel&     = default;
  auto operator=(PositionObservationModel&&) noexcept -> PositionObservationModel& = default;
  virtual ~PositionObservationModel() TEST_REMOVE_FINAL                            = default;

  /// \brief Construct a new PositionObservationModel given the measurement and its covariance
  /// \param[in] vec  Measurement vector z = [x, y]'
  /// \param[in] cov  Measurement covariance R
  explicit PositionObservationModel(const MeasurementVec& vec, const MeasurementCov& cov)
      : BaseExtendedObservationModel{vec, cov}
  {
  }

  /// \brief Construct a new PositionObservationModel given the measurement, its covariance and a sensor mounting pose
  /// \param[in] vec  Measurement vector z = [x, y]'
  /// \param[in] cov  Measurement covariance R
  /// \param[in] pose Static SE(2) sensor mounting pose relative to the tracking frame
  explicit PositionObservationModel(const MeasurementVec&                                    vec,
                                    const MeasurementCov&                                    cov,
                                    const typename BaseExtendedObservationModel::SensorPose& pose)
      : BaseExtendedObservationModel{vec, cov, pose}
  {
  }

  /// \brief Predict the measurement h(x) = [x, y]' for the given sensor-frame state
  /// \param[in] state  Sensor-frame state vector the measurement is predicted for
  /// \return MeasurementVec  Predicted measurement
  auto predictMeasurementSensorFrame(const StateVec& state) const -> MeasurementVec
  {
    MeasurementVec predicted{};
    predicted.at_unsafe(MEAS_X) = state.at_unsafe(StateDef_::X);
    predicted.at_unsafe(MEAS_Y) = state.at_unsafe(StateDef_::Y);
    return predicted;
  }

  /// \brief Compute the sensor-frame-local measurement Jacobian at the given sensor-frame state
  ///
  /// h(x) is affine in the sensor-frame position, so the local Jacobian is the identity; the base
  /// class chains the constant mounting rotation onto the tracking-frame state columns afterwards.
  ///
  /// \param[out] jacobian  The measurement Jacobian to be filled (sensor-frame-local partials)
  /// \param[in]  state     Sensor-frame state vector the Jacobian is linearized at (unused, model is linear)
  void computeJacobianSensorFrame(JacobianMatrix& jacobian, const StateVec& state) const
  {
    static_cast<void>(state);
    jacobian.setZeros();
    jacobian.at_unsafe(MEAS_X, StateDef_::X) = static_cast<value_type>(1.0);
    jacobian.at_unsafe(MEAS_Y, StateDef_::Y) = static_cast<value_type>(1.0);
  }
};

} // namespace observation
} // namespace tracking

#endif // B2C3D4E5_7F8A_4B9C_A0D1_2E3F4A5B6C7D
