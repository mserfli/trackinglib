#ifndef FA800472_29A5_4B6F_90A0_4283A0D513D6
#define FA800472_29A5_4B6F_90A0_4283A0D513D6

#include "base/first_include.h"                     // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.h"     // IWYU pragma: keep
#include "math/linalg/matrix.h"
#include "motion/imotion_model.h"


namespace tracking
{
namespace motion
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen

// forward declaration to avoid recursive inclusion
template <typename CovarianceMatrixPolicy_>
class MotionModelCA;

struct StateDefCV
{
  enum _
  {
    X = 0,
    VX,
    Y,
    VY,
    NUM_STATE_VARIABLES
  };
};

template <typename CovarianceMatrixPolicy_>
class MotionModelCV TEST_REMOVE_FINAL
    : public StateDefCV
    , public ExtendedMotionModel<MotionModelCV<CovarianceMatrixPolicy_>, MotionModelTraits<CovarianceMatrixPolicy_, StateDefCV>>
{
public:
  enum NoiseDef
  {
    Q_VX = 0,
    Q_VY,
    NUM_PROC_NOISE_VARIABLES
  };

  using instance_type           = MotionModelCV<CovarianceMatrixPolicy_>;
  using instance_trait          = MotionModelTraits<CovarianceMatrixPolicy_, StateDefCV>;
  using BaseExtendedMotionModel = ExtendedMotionModel<instance_type, instance_trait>;
  using value_type              = typename instance_trait::value_type;
  using StateVec                = typename BaseExtendedMotionModel::StateVec;
  using StateCov                = typename BaseExtendedMotionModel::StateCov;

  using StateMatrix               = math::SquareMatrix<value_type, NUM_STATE_VARIABLES>;
  using ProcessNoiseDiagMatrix    = math::DiagonalMatrix<value_type, NUM_PROC_NOISE_VARIABLES>;
  using ProcessNoiseMappingMatrix = math::Matrix<value_type, NUM_STATE_VARIABLES, NUM_PROC_NOISE_VARIABLES>;

  using EgoMotionType          = typename BaseExtendedMotionModel::EgoMotionType;
  using EgoMotionMappingMatrix = math::Matrix<value_type, NUM_STATE_VARIABLES, EgoMotionType::DS_NUM_VARIABLES>;

  static constexpr sint32 NUM_AUG_PROC_NOISE_VARIABLES =
      NUM_PROC_NOISE_VARIABLES + static_cast<sint32>(EgoMotionType::DS_NUM_VARIABLES);
  using AugmentedProcessNoiseDiagMatrix    = math::DiagonalMatrix<value_type, NUM_AUG_PROC_NOISE_VARIABLES>;
  using AugmentedProcessNoiseMappingMatrix = math::Matrix<value_type, NUM_STATE_VARIABLES, NUM_AUG_PROC_NOISE_VARIABLES>;

  // rule of 5 declarations
  MotionModelCV()                                            = default;
  MotionModelCV(const MotionModelCV&)                        = default;
  MotionModelCV(MotionModelCV&&) noexcept                    = default;
  auto operator=(const MotionModelCV&) -> MotionModelCV&     = default;
  auto operator=(MotionModelCV&&) noexcept -> MotionModelCV& = default;
  virtual ~MotionModelCV() TEST_REMOVE_FINAL                 = default;

  /// \brief Construct a new CV given the vector and the covariance matrix
  /// \param[in] vec
  /// \param[in] cov
  explicit MotionModelCV(const StateVec& vec, const StateCov& cov);

  /// \brief Read access to x velocity
  /// \return value_type
  auto getVx() const -> value_type final { return this->operator[](StateDefCV::VX); }
  /// \brief Read access to y velocity
  /// \return value_type
  auto getVy() const -> value_type final { return this->operator[](StateDefCV::VY); }
  /// \brief Read access to x acceleration
  /// \return value_type
  auto getAx() const -> value_type final { return static_cast<value_type>(0.0); }
  /// \brief Read access to y acceleration
  /// \return value_type
  auto getAy() const -> value_type final { return static_cast<value_type>(0.0); }
  // ... all the other virtual functions

  /// \brief Creates a CV model based on a CA model
  /// \param[in] other  The CA model
  void convertFrom(const MotionModelCA<CovarianceMatrixPolicy_>& other);

  /// \brief Compute Ge and Go for compensation of the covariance
  /// \param[out] Ge        The propagated errors of the ego motion to the state space
  /// \param[out] Go        The transformation of the state caused by the ego motion
  /// \param[in]  egoMotion The known egoMotion from last state to predicted state
  TEST_VIRTUAL void computeEgoMotionCompensationMatrices(EgoMotionMappingMatrix& Ge,
                                                         StateMatrix&            Go,
                                                         const EgoMotionType&    egoMotion);

  /// \brief Compensate the state with the given ego motion
  /// \param[in]  egoMotion The known egoMotion from last state to predicted state
  TEST_VIRTUAL void compensateState(const EgoMotionType& egoMotion);

  /// \brief Apply the state transition from k to k+1 defined by the process model
  /// \param[in] dt         The delta time from last state to predicted state
  void applyProcessModel(const value_type dt);

  /// \brief Compute matrix A using the 1st order linearisation of the state transition from k to k+1
  /// \param[out] A         Linearisation of the state transition
  /// \param[in]  dt        The delta time from last state to predicted state
  void computeA(StateMatrix& A, const value_type dt) const;

  /// \brief Compute the diagonal process noise matrix Q.
  /// \param[out] Q         The process noise
  /// \param[in]  dt        The delta time from last state to predicted state
  static void computeQ(ProcessNoiseDiagMatrix& Q, const value_type dt);

  /// \brief Compute the matrix G to map the diagonoal process noise to the full state
  /// \param[out] G         The transformation of the process noise to the full state space
  /// \param[in]  dt        The delta time from last state to predicted state
  static void computeG(ProcessNoiseMappingMatrix& G, const value_type dt);
};

} // namespace motion
} // namespace tracking

#endif // FA800472_29A5_4B6F_90A0_4283A0D513D6
