#ifndef A1B2C3D4_6E7F_4A8B_9C0D_1E2F3A4B5C6D
#define A1B2C3D4_6E7F_4A8B_9C0D_1E2F3A4B5C6D

#include "base/first_include.h" // IWYU pragma: keep
#include "base/require_abstract_intf.h"
#include "math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"
#include "motion/state_def_traits.h"
#include "motion/state_mem.h"
#include "observation/iobservation_model.h"

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
/// Concrete models implement predictMeasurement() (the observation function h(x)) and
/// computeJacobian() (H = dh/dx) and may shadow computeInnovation() for measurement components
/// that need special innovation handling (e.g. angle wrapping).
///
/// \tparam ObservationModel_       The underlying concrete ObservationModel (CRTP)
/// \tparam ObservationModelTrait_  ObservationModelTraits instantiation describing the model
template <typename ObservationModel_, typename ObservationModelTrait_>
class ExtendedObservationModel
    : public IObservationModel<typename ObservationModelTrait_::CovarianceMatrixPolicy>
    , public motion::StateMem<typename ObservationModelTrait_::CovarianceMatrixPolicy, ObservationModelTrait_::DimZ>
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

  // rule of 5 declarations
  ExtendedObservationModel()          = default;
  virtual ~ExtendedObservationModel() = default;

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
                                     const std::initializer_list<value_type>& d) -> std::enable_if_t<T::is_factored, MeasurementCov>
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

  /// \brief Read access to the measurement dimension
  /// \return sint32  Number of measurement components
  auto getDim() const -> sint32 final { return DimZ; }

  /// \brief Predict the measurement h(x) for the given state
  /// \param[in] state  State vector (state space) the measurement is predicted for
  /// \return MeasurementVec  Predicted measurement h(x)
  virtual auto predictMeasurement(const StateVec& state) const -> MeasurementVec = 0;

  /// \brief Compute the measurement Jacobian H = dh/dx at the given state
  /// \param[out] jacobian  The measurement Jacobian to be filled
  /// \param[in]  state     State vector (state space) the Jacobian is linearized at
  virtual void computeJacobian(JacobianMatrix& jacobian, const StateVec& state) const = 0;

  /// \brief Compute the innovation nu = z - h(x)
  ///
  /// Default component-wise difference. Concrete models may shadow this method for measurement
  /// components that need special innovation handling (e.g. angle wrapping for bearings);
  /// generic::Update resolves the call statically on the concrete model type.
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
      : BaseIObservationModel{}
      , BaseStateMem{vec, cov}
  {
    assert(cov.determinant() > 0);
  }
};

} // namespace observation
} // namespace tracking

#endif // A1B2C3D4_6E7F_4A8B_9C0D_1E2F3A4B5C6D
