#ifndef EB85CC72_56D5_4D88_89CF_98C6580F5B61
#define EB85CC72_56D5_4D88_89CF_98C6580F5B61

#include "base/first_include.h"
#include "env/ego_motion.h"
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/matrix.h"
#include "motion/generic_predict.hpp"
#include "motion/imotion_model.h"

namespace tracking
{
namespace motion
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen

// forward declaration to avoid recursive inclusion
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
class MotionModelCV;

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
class MotionModelCA
    : public ExtendedMotionModel<MotionModelCA<CovarianceMatrixType, FloatType>, CovarianceMatrixType, FloatType, 6>
    , public generic::Predict<MotionModelCA<CovarianceMatrixType, FloatType>, FloatType, CovarianceMatrixType>
{
public:
  enum NoiseDef
  {
    Q_AX = 0,
    Q_AY,
    NUM_PROC_NOISE_VARIABLES
  };

  enum StateDef
  {
    X = 0,
    VX,
    AX,
    Y,
    VY,
    AY,
    NUM_STATE_VARIABLES
  };

  using instance_type              = MotionModelCA<CovarianceMatrixType, FloatType>;
  using super_extended_mm_type     = ExtendedMotionModel<instance_type, CovarianceMatrixType, FloatType, 6>;
  using super_generic_predict_type = generic::Predict<instance_type, FloatType, CovarianceMatrixType>;
  using StateVec                   = typename super_extended_mm_type::StateVec;
  using StateCov                   = typename super_extended_mm_type::StateCov;

  using StateMatrix               = math::SquareMatrix<FloatType, NUM_STATE_VARIABLES>;
  using ProcessNoiseDiagMatrix    = math::DiagonalMatrix<FloatType, NUM_PROC_NOISE_VARIABLES>;
  using ProcessNoiseMappingMatrix = math::Matrix<FloatType, NUM_STATE_VARIABLES, NUM_PROC_NOISE_VARIABLES>;

  using EgoMotion              = env::EgoMotion<FloatType>;
  using EgoMotionMappingMatrix = math::Matrix<FloatType, NUM_STATE_VARIABLES, EgoMotion::DS_NUM_VARIABLES>;

  static constexpr sint32 NUM_AUG_PROC_NOISE_VARIABLES = NUM_PROC_NOISE_VARIABLES + EgoMotion::DS_NUM_VARIABLES;
  using AugmentedProcessNoiseDiagMatrix                = math::DiagonalMatrix<FloatType, NUM_AUG_PROC_NOISE_VARIABLES>;
  using AugmentedProcessNoiseMappingMatrix = math::Matrix<FloatType, NUM_STATE_VARIABLES, NUM_AUG_PROC_NOISE_VARIABLES>;

  // rule of 5 declarations
  MotionModelCA()                         = default;
  MotionModelCA(const MotionModelCA&)     = default;
  MotionModelCA(MotionModelCA&&) noexcept = default;
  auto operator=(const MotionModelCA&) -> MotionModelCA& = default;
  auto operator=(MotionModelCA&&) noexcept -> MotionModelCA& = default;
  virtual ~MotionModelCA()                                   = default;

  /// \brief Construct a new CA given the vector and the covariance matrix
  /// \param[in] vec
  /// \param[in] cov
  explicit MotionModelCA(const StateVec& vec, const StateCov& cov);

  /// \brief Read access to x velocity
  /// \return FloatType
  auto getVx() const -> FloatType final { return this->operator[](StateDef::VX); }
  /// \brief Read access to y velocity
  /// \return FloatType
  auto getVy() const -> FloatType final { return this->operator[](StateDef::VY); }
  /// \brief Read access to x acceleration
  /// \return FloatType
  auto getAx() const -> FloatType final { return this->operator[](StateDef::AX); }
  /// \brief Read access to y acceleration
  /// \return FloatType
  auto getAy() const -> FloatType final { return this->operator[](StateDef::AY); }
  // ... all the other virtual functions

  /// \brief Predicts the underlying MotionModel with the given filter (includes ego motion compensation)
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void predict(const FloatType                        dt,
               const filter::KalmanFilter<FloatType>& filter,
               const env::EgoMotion<FloatType>&       egoMotion) final;

  void predict(const FloatType                             dt,
               const filter::InformationFilter<FloatType>& filter,
               const env::EgoMotion<FloatType>&            egoMotion) final;

  /// \brief Creates a CA model based on a CV model
  /// \param[in] other  The CV model
  void convertFrom(const MotionModelCV<CovarianceMatrixType, FloatType>& other);

  /// \brief Compensates the state according to the known ego motion and calculates Ge and Go for compensation of the covariance
  /// \param[out] Ge        The propagated errors of the ego motion to the state space
  /// \param[out] Go        The transformation of the state caused by the ego motion
  /// \param[in]  egoMotion The known egoMotion from last state to predicted state
  TEST_VIRTUAL void compensateEgoMotion(EgoMotionMappingMatrix& Ge, StateMatrix& Go, const EgoMotion& egoMotion);

  /// \brief Apply the state transition from k to k+1 defined by the process model
  /// \param[in] dt         The delta time from last state to predicted state
  void applyProcessModel(const FloatType dt);

  /// \brief Compute matrix A using the 1st order linearisation of the state transition from k to k+1
  /// \param[out] A         Linearisation of the state transition
  /// \param[in]  dt        The delta time from last state to predicted state
  void computeA(StateMatrix& A, const FloatType dt) const;

  /// \brief Compute the diagonal process noise matrix Q.
  /// \param[out] Q         The process noise
  /// \param[in]  dt        The delta time from last state to predicted state
  static void computeQ(ProcessNoiseDiagMatrix& Q, const FloatType dt);

  /// \brief Compute the matrix G to map the diagonoal process noise to the full state
  /// \param[out] G         The transformation of the process noise to the full state space
  /// \param[in]  dt        The delta time from last state to predicted state
  static void computeG(ProcessNoiseMappingMatrix& G, const FloatType dt);
};

} // namespace motion
} // namespace tracking

#endif // EB85CC72_56D5_4D88_89CF_98C6580F5B61
