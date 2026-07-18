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

  /// \brief Predict the measurement h(x) = [vx, vy]' for the given state
  /// \param[in] state  State vector the measurement is predicted for
  /// \return MeasurementVec  Predicted measurement
  auto predictMeasurement(const StateVec& state) const -> MeasurementVec final
  {
    MeasurementVec predicted{};
    predicted.at_unsafe(MEAS_VX) = state.at_unsafe(StateDef_::VX);
    predicted.at_unsafe(MEAS_VY) = state.at_unsafe(StateDef_::VY);
    return predicted;
  }

  /// \brief Compute the measurement Jacobian H = dh/dx at the given state
  /// \param[out] jacobian  The measurement Jacobian to be filled
  /// \param[in]  state     State vector the Jacobian is linearized at (unused, model is linear)
  void computeJacobian(JacobianMatrix& jacobian, const StateVec& state) const final
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
