#ifndef A1B2C3D4_6E7F_4A8B_9C0D_1E2F3A4B5C6D
#define A1B2C3D4_6E7F_4A8B_9C0D_1E2F3A4B5C6D

#include "base/first_include.h" // IWYU pragma: keep
#include "base/require_abstract_intf.h"
#include "env/ego_motion.h"
#include "math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"
#include "motion/state_def_traits.h"
#include "motion/state_mem.h"
#include "observation/contracts/observation_model_intf.h"
#include "observation/iobservation_model.h"
#include "observation/sensor_mounting_pose.h"

namespace tracking
{
namespace observation
{

/// \brief Abstract ObservationModel with known dimensions, keeping the model memory for the
///        measurement vector z and the measurement covariance R
///
/// CRTP base for concrete observation models, mirroring motion::ExtendedMotionModel. It reuses
/// motion::StateMem for the measurement memory: getVec() returns the measurement vector z and
/// getCov() returns the measurement covariance R.
///
/// Template-Method pattern: this base owns predictMeasurement()/computeJacobian() (called by
/// generic::Update) and unconditionally applies the sensor mounting pose transform via
/// toSensorFrame()/applyJacobianFrameRotation() before delegating to sensor-frame-only hooks that
/// concrete models must provide (no vtable entry is involved — resolved statically via the CRTP
/// derived() downcast):
///   - predictMeasurementSensorFrame(const StateVec&, const EgoMotionType&) const -> MeasurementVec  (h(x) in sensor frame)
///   - computeJacobianSensorFrame(JacobianMatrix&, const StateVec&, const EgoMotionType&) const      (dh/dx in sensor frame)
/// and may shadow computeInnovation() for measurement components that need special innovation
/// handling (e.g. angle wrapping). Because the base applies the transform, a concrete model cannot
/// forget it or apply it inconsistently between prediction and Jacobian; the mandatory two hooks
/// are enforced at compile time by contract::ObservationModelIntf (this class's CRTP contract base).
///
/// \tparam ObservationModel_       The underlying concrete ObservationModel (CRTP)
/// \tparam ObservationModelTrait_  ObservationModelTraits instantiation describing the model
template <typename ObservationModel_, typename ObservationModelTrait_>
class ExtendedObservationModel
    : public IObservationModel<typename ObservationModelTrait_::CovarianceMatrixPolicy>
    , public motion::StateMem<typename ObservationModelTrait_::CovarianceMatrixPolicy, ObservationModelTrait_::DimZ>
    , public contract::ObservationModelIntf<ObservationModel_>
    , public base::contract::RequireAbstractIntf<ExtendedObservationModel<ObservationModel_, ObservationModelTrait_>>
{
public:
  using value_type             = typename ObservationModelTrait_::value_type;
  using StateDef               = typename ObservationModelTrait_::StateDef;
  using CovarianceMatrixPolicy = typename ObservationModelTrait_::CovarianceMatrixPolicy;
  using BaseIObservationModel  = IObservationModel<CovarianceMatrixPolicy>;

  /// \brief Measurement dimension (number of measurement components)
  static constexpr sint32 DimZ = ObservationModelTrait_::DimZ;
  /// \brief State dimension of the observed StateDef
  static constexpr sint32 DimX = motion::state_dimension_v<StateDef>;

  using BaseStateMem   = motion::StateMem<CovarianceMatrixPolicy, DimZ>;
  using MeasurementVec = typename BaseStateMem::StateVec;
  using MeasurementCov = typename BaseStateMem::StateCov;
  using StateVec       = math::Vector<value_type, DimX>;
  using JacobianMatrix = math::Matrix<value_type, DimZ, DimX>;
  using SensorPose     = SensorMountingPose<value_type>;
  using EgoMotionType  = env::EgoMotion<CovarianceMatrixPolicy>;

  // rule of 5 declarations
  ExtendedObservationModel() = default;
  /// \brief Pure virtual destructor keeping this CRTP layer abstract (RequireAbstractIntf
  ///        contract) without introducing vtable entries for the statically dispatched hooks
  virtual ~ExtendedObservationModel() = 0;

  /// \brief Create measurement vector from initializer list
  /// \param[in] list  Initializer list with measurement values
  /// \return MeasurementVec
  static auto MeasurementVecFromList(const std::initializer_list<value_type>& list) -> MeasurementVec
  {
    return MeasurementVec::FromList(list);
  }

  /// \brief Create measurement covariance from initializer list
  /// \param[in] list  Nested initializer list with covariance values
  /// \return MeasurementCov
  static auto MeasurementCovFromList(const std::initializer_list<std::initializer_list<value_type>>& list) -> MeasurementCov
  {
    if constexpr (CovarianceMatrixPolicy::is_factored)
    {
      return math::conversions::CovarianceMatrixFactoredFromList<value_type, DimZ>(list);
    }
    else
    {
      return MeasurementCov::FromList(list);
    }
  }

  /// \brief Create factored measurement covariance from initializer list
  /// \tparam T  SFINAE helper defaulting to CovarianceMatrixPolicy
  /// \param[in] u Nested initializer list for the upper triangular U matrix
  /// \param[in] d Flat initializer list for the diagonal D matrix
  /// \return MeasurementCov
  template <typename T = CovarianceMatrixPolicy>
  static auto MeasurementCovFromList(const std::initializer_list<std::initializer_list<value_type>>& u,
                                     const std::initializer_list<value_type>&                        d)
      -> std::enable_if_t<T::is_factored, MeasurementCov>
  {
    return MeasurementCov::FromList(u, d);
  }

  /// \brief Create complete ObservationModel from initializer lists
  /// \param[in] vecList  Initializer list for the measurement vector
  /// \param[in] covList  Nested initializer list for the measurement covariance matrix
  /// \return ObservationModel_ instance
  static auto FromLists(const std::initializer_list<value_type>&                        vecList,
                        const std::initializer_list<std::initializer_list<value_type>>& covList) -> ObservationModel_
  {
    auto vec = MeasurementVecFromList(vecList);
    auto cov = MeasurementCovFromList(covList);
    return ObservationModel_{vec, cov};
  }

  /// \brief Create complete ObservationModel from initializer lists and a sensor mounting pose
  /// \param[in] vecList  Initializer list for the measurement vector
  /// \param[in] covList  Nested initializer list for the measurement covariance matrix
  /// \param[in] pose     Static SE(2) sensor mounting pose relative to the tracking frame
  /// \return ObservationModel_ instance
  static auto FromLists(const std::initializer_list<value_type>&                        vecList,
                        const std::initializer_list<std::initializer_list<value_type>>& covList,
                        const SensorPose&                                               pose) -> ObservationModel_
  {
    auto vec = MeasurementVecFromList(vecList);
    auto cov = MeasurementCovFromList(covList);
    return ObservationModel_{vec, cov, pose};
  }

  /// \brief Read access to the measurement dimension
  /// \return sint32  Number of measurement components
  auto getDim() const -> sint32 final { return DimZ; }

  /// \brief Read access to the sensor mounting pose
  /// \return SensorPose  Static SE(2) sensor mounting pose relative to the tracking frame
  auto getSensorPose() const -> const SensorPose& { return _sensorPose; }

  /// \brief Predict the measurement h(x) for the given tracking-frame state
  ///
  /// Transforms the state into the sensor frame via toSensorFrame() and delegates to the derived
  /// model's predictMeasurementSensorFrame() hook, which operates purely in the sensor frame. This
  /// guarantees every concrete model applies the sensor mounting pose consistently, since the
  /// transform lives here rather than being repeated (or forgotten) per model.
  ///
  /// \param[in] state      Tracking-frame state vector the measurement is predicted for
  /// \param[in] egoMotion  Ego motion of the sensor platform
  /// \return MeasurementVec  Predicted measurement h(x)
  [[nodiscard]] auto predictMeasurement(const StateVec& state, const EgoMotionType& egoMotion) const -> MeasurementVec
  {
    return derived().predictMeasurementSensorFrame(toSensorFrame(state), egoMotion);
  }

  /// \brief Compute the measurement Jacobian H = dh/dx for the given tracking-frame state
  ///
  /// Transforms the state into the sensor frame, delegates to the derived model's
  /// computeJacobianSensorFrame() hook to fill the sensor-frame-local partials, then chains the
  /// constant mounting rotation back onto the tracking-frame state columns via
  /// applyJacobianFrameRotation(). See predictMeasurement() for the rationale.
  ///
  /// \param[out] jacobian  The measurement Jacobian to be filled
  /// \param[in]  state     Tracking-frame state vector the Jacobian is linearized at
  /// \param[in]  egoMotion Ego motion of the sensor platform
  void computeJacobian(JacobianMatrix& jacobian, const StateVec& state, const EgoMotionType& egoMotion) const
  {
    derived().computeJacobianSensorFrame(jacobian, toSensorFrame(state), egoMotion);
    applyJacobianFrameRotation(jacobian);
  }

  /// \brief Compute the innovation nu = z - h(x)
  ///
  /// Default component-wise difference. Concrete models may shadow this method for measurement
  /// components that need special innovation handling (e.g. angle wrapping for bearings);
  /// generic::Update resolves the call statically on the concrete model type — the same static
  /// mechanism as the predictMeasurement/computeJacobian contract (see class description).
  ///
  /// \param[in] measurement  Measurement vector z
  /// \param[in] predicted    Predicted measurement h(x)
  /// \return MeasurementVec  Innovation nu = z - h(x)
  auto computeInnovation(const MeasurementVec& measurement, const MeasurementVec& predicted) const -> MeasurementVec
  {
    return MeasurementVec{measurement - predicted};
  }

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround to keep following idententation
  // clang-format on

  // rule of 5 declarations (remaining declarations are protected according to A12-8-6)
  ExtendedObservationModel(const ExtendedObservationModel& other)                    = default;
  ExtendedObservationModel(ExtendedObservationModel&&) noexcept                      = default;
  auto operator=(const ExtendedObservationModel& other) -> ExtendedObservationModel& = default;
  auto operator=(ExtendedObservationModel&&) noexcept -> ExtendedObservationModel&   = default;

  /// \brief Construct a new Extended Observation Model object
  /// \param[in] vec  Measurement vector z
  /// \param[in] cov  Measurement covariance R
  explicit ExtendedObservationModel(const MeasurementVec& vec, const MeasurementCov& cov)
      : ExtendedObservationModel{vec, cov, SensorPose{}}
  {
  }

  /// \brief Construct a new Extended Observation Model object with a non-identity sensor mounting pose
  /// \param[in] vec  Measurement vector z
  /// \param[in] cov  Measurement covariance R
  /// \param[in] pose Static SE(2) sensor mounting pose relative to the tracking frame
  explicit ExtendedObservationModel(const MeasurementVec& vec, const MeasurementCov& cov, const SensorPose& pose)
      : BaseIObservationModel{}
      , BaseStateMem{vec, cov}
      , _sensorPose{pose}
  {
    assert(cov.determinant() > 0);
  }

private:
  /// \brief CRTP downcast to the concrete derived observation model
  /// \return const ObservationModel_&  The concrete model this base is instantiated for
  auto derived() const -> const ObservationModel_& { return static_cast<const ObservationModel_&>(*this); }

  /// \brief Transform every present state component (position/velocity/acceleration) into the sensor frame
  ///
  /// Driven by the existing motion::has_position_v/has_velocity_v/has_acceleration_v traits so no
  /// per-model trait is needed. A pair the StateDef doesn't provide is simply skipped.
  ///
  /// \param[in] state  Tracking-frame state vector
  /// \return StateVec  State vector with all present position/velocity/acceleration pairs expressed in the sensor frame
  [[nodiscard]] auto toSensorFrame(const StateVec& state) const -> StateVec
  {
    StateVec transformed = state;
    if constexpr (motion::has_position_v<StateDef>)
    {
      const auto p = getSensorPose().positionToSensorFrame(state.at_unsafe(StateDef::X), state.at_unsafe(StateDef::Y));
      transformed.at_unsafe(StateDef::X) = p.x();
      transformed.at_unsafe(StateDef::Y) = p.y();
    }
    if constexpr (motion::has_velocity_v<StateDef>)
    {
      const auto v = getSensorPose().directionToSensorFrame(state.at_unsafe(StateDef::VX), state.at_unsafe(StateDef::VY));
      transformed.at_unsafe(StateDef::VX) = v.x();
      transformed.at_unsafe(StateDef::VY) = v.y();
    }
    if constexpr (motion::has_acceleration_v<StateDef>)
    {
      const auto a = getSensorPose().directionToSensorFrame(state.at_unsafe(StateDef::AX), state.at_unsafe(StateDef::AY));
      transformed.at_unsafe(StateDef::AX) = a.x();
      transformed.at_unsafe(StateDef::AY) = a.y();
    }
    return transformed;
  }

  /// \brief Chain the constant sensor mounting rotation onto every present Jacobian column pair
  ///
  /// Post-multiplies each present (position/velocity/acceleration) column pair by R(theta)^T,
  /// turning sensor-frame-local partials (filled by computeJacobianSensorFrame()) into partials
  /// w.r.t. the underlying tracking-frame state columns. A pair with all-zero local partials (e.g.
  /// a component the concrete model doesn't use) rotates to zero, so applying this unconditionally
  /// to every present pair is always safe.
  ///
  /// \param[in,out] jacobian  Jacobian matrix updated in place
  void applyJacobianFrameRotation(JacobianMatrix& jacobian) const
  {
    if constexpr (motion::has_position_v<StateDef>)
    {
      getSensorPose().rotateJacobianColumns(jacobian, StateDef::X, StateDef::Y);
    }
    if constexpr (motion::has_velocity_v<StateDef>)
    {
      getSensorPose().rotateJacobianColumns(jacobian, StateDef::VX, StateDef::VY);
    }
    if constexpr (motion::has_acceleration_v<StateDef>)
    {
      getSensorPose().rotateJacobianColumns(jacobian, StateDef::AX, StateDef::AY);
    }
  }

  SensorPose _sensorPose{};
};

// out-of-class definition of the pure virtual destructor (keeps the class abstract while
// still providing the destructor implementation required by the concrete models)
template <typename ObservationModel_, typename ObservationModelTrait_>
ExtendedObservationModel<ObservationModel_, ObservationModelTrait_>::~ExtendedObservationModel() = default;

} // namespace observation
} // namespace tracking

#endif // A1B2C3D4_6E7F_4A8B_9C0D_1E2F3A4B5C6D
