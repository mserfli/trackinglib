#ifndef EB85CC72_56D5_4D88_89CF_98C6580F5B61
#define EB85CC72_56D5_4D88_89CF_98C6580F5B61

#include "base/first_include.h"
#include "env/ego_motion.h"
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/matrix.h"
#include "motion/generic_predict.h"
#include "motion/imotion_model.h"

namespace tracking
{
namespace motion
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen

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

  using instance_type = MotionModelCA<CovarianceMatrixType, FloatType>;
  using super_extended_mm_type = ExtendedMotionModel<instance_type, CovarianceMatrixType, FloatType, 6>;
  using super_generic_predict_type = generic::Predict<instance_type, FloatType, CovarianceMatrixType>;
  using StateVec = typename super_extended_mm_type::StateVec;
  using StateCov = typename super_extended_mm_type::StateCov;

  using StateMatrix = math::SquareMatrix<FloatType, NUM_STATE_VARIABLES>;
  using ProcessNoiseDiagMatrix = math::DiagonalMatrix<FloatType, NUM_PROC_NOISE_VARIABLES>;
  using ProcessNoiseMappingMatrix = math::Matrix<FloatType, NUM_STATE_VARIABLES, NUM_PROC_NOISE_VARIABLES>;

  using EgoMotion = env::EgoMotion<FloatType>;
  using EgoMotionMappingMatrix = math::Matrix<FloatType, NUM_STATE_VARIABLES, EgoMotion::DS_NUM_VARIABLES>;

  static constexpr sint32 NUM_AUG_PROC_NOISE_VARIABLES = NUM_PROC_NOISE_VARIABLES + EgoMotion::DS_NUM_VARIABLES;
  using AugmentedProcessNoiseDiagMatrix = math::DiagonalMatrix<FloatType, NUM_AUG_PROC_NOISE_VARIABLES>;
  using AugmentedProcessNoiseMappingMatrix = math::Matrix<FloatType, NUM_STATE_VARIABLES, NUM_AUG_PROC_NOISE_VARIABLES>;

  // rule of 5 declarations
  MotionModelCA() = default;
  MotionModelCA(const MotionModelCA&) = default;
  MotionModelCA(MotionModelCA&&) noexcept = default;
  auto operator=(const MotionModelCA&) -> MotionModelCA& = default;
  auto operator=(MotionModelCA&&) noexcept -> MotionModelCA& = default;
  ~MotionModelCA() final = default;

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

  /// \brief Compensates the state according to the known ego motion and calculates Ge and Go for compensation of the covariance
  /// \param[out] Ge        The propagated errors of the ego motion to the state space
  /// \param[out] Go        The transformation of the state caused by the ego motion
  /// \param[in]  egoMotion The known egoMotion from last state to predicted state
  void compensateEgoMotion(EgoMotionMappingMatrix& Ge, StateMatrix& Go, const EgoMotion& egoMotion);

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

  void convertFrom(const MotionModelCV<CovarianceMatrixType, FloatType>& other);
};

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::predict(const FloatType                        dt,
                                                             const filter::KalmanFilter<FloatType>& filter,
                                                             const env::EgoMotion<FloatType>&       egoMotion)
{
  super_generic_predict_type::run(dt, filter, egoMotion);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::compensateEgoMotion(EgoMotionMappingMatrix& Ge,
                                                                         StateMatrix&            Go,
                                                                         const EgoMotion&        egoMotion)
{
  FloatType& x = this->operator[](StateDef::X);
  FloatType& y = this->operator[](StateDef::Y);
  FloatType& vx = this->operator[](StateDef::VX);
  FloatType& vy = this->operator[](StateDef::VY);
  FloatType& ax = this->operator[](StateDef::AX);
  FloatType& ay = this->operator[](StateDef::AY);

  const FloatType sinDeltaPsiEgo = egoMotion.getDisplacementCog().sinDeltaPsi;
  const FloatType cosDeltaPsiEgo = egoMotion.getDisplacementCog().cosDeltaPsi;
  const FloatType deltaXEgo = egoMotion.getDisplacementCog().vec[EgoMotion::DS_X];
  const FloatType deltaYEgo = egoMotion.getDisplacementCog().vec[EgoMotion::DS_Y];
  const FloatType distCog2Ego = egoMotion.getGeometry().distCog2Ego;

  Go.setZeros();
  Go(X, X) = cosDeltaPsiEgo;
  Go(X, Y) = sinDeltaPsiEgo;
  Go(Y, X) = -sinDeltaPsiEgo;
  Go(Y, Y) = cosDeltaPsiEgo;
  Go(VX, VX) = cosDeltaPsiEgo;
  Go(VX, VY) = sinDeltaPsiEgo;
  Go(VY, VX) = -sinDeltaPsiEgo;
  Go(VY, VY) = cosDeltaPsiEgo;
  Go(AX, AX) = cosDeltaPsiEgo;
  Go(AX, AY) = sinDeltaPsiEgo;
  Go(AY, AX) = -sinDeltaPsiEgo;
  Go(AY, AY) = cosDeltaPsiEgo;

  const FloatType x0 = -deltaYEgo + y;
  const FloatType x1 = deltaXEgo - distCog2Ego - x;
  Ge.setZeros();
  Ge(X, EgoMotion::DS_X) = -cosDeltaPsiEgo;
  Ge(X, EgoMotion::DS_Y) = -sinDeltaPsiEgo;
  Ge(X, EgoMotion::DS_PSI) = (x0 * cosDeltaPsiEgo) + (x1 * sinDeltaPsiEgo);
  Ge(Y, EgoMotion::DS_X) = sinDeltaPsiEgo;
  Ge(Y, EgoMotion::DS_Y) = cosDeltaPsiEgo;
  Ge(Y, EgoMotion::DS_PSI) = -(x0 * sinDeltaPsiEgo) + (x1 * cosDeltaPsiEgo);
  Ge(VX, EgoMotion::DS_PSI) = -(vx * sinDeltaPsiEgo) + (vy * cosDeltaPsiEgo);
  Ge(VY, EgoMotion::DS_PSI) = -(vx * cosDeltaPsiEgo) - (vy * sinDeltaPsiEgo);
  Ge(AX, EgoMotion::DS_PSI) = -(ax * sinDeltaPsiEgo) + (ay * cosDeltaPsiEgo);
  Ge(AY, EgoMotion::DS_PSI) = -(ax * cosDeltaPsiEgo) - (ay * sinDeltaPsiEgo);

  // translate and rotate position
  egoMotion.compensatePosition(x, y, x, y);
  // rotate velocity and acceleration
  egoMotion.compensateDirection(vx, vy, vx, vy);
  egoMotion.compensateDirection(ax, ay, ax, ay);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::applyProcessModel(const FloatType dt)
{
  const StateVec& stateVec = this->getVec();
  const auto      halfDtSq = static_cast<FloatType>(0.5) * dt * dt;

  this->operator[](StateDef::X) += dt * stateVec[StateDef::VX] + halfDtSq * stateVec[StateDef::AX];
  this->operator[](StateDef::Y) += dt * stateVec[StateDef::VY] + halfDtSq * stateVec[StateDef::AY];
  this->operator[](StateDef::VX) += dt * stateVec[StateDef::AX];
  this->operator[](StateDef::VY) += dt * stateVec[StateDef::AY];
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::computeA(StateMatrix& A, const FloatType dt) const
{
  const auto halfDtSq = static_cast<FloatType>(0.5) * dt * dt;

  A.setIdentity();
  A(StateDef::X, StateDef::AX) = halfDtSq;
  A(StateDef::X, StateDef::VX) = dt;
  A(StateDef::VX, StateDef::AX) = dt;

  A(StateDef::Y, StateDef::AY) = halfDtSq;
  A(StateDef::Y, StateDef::VY) = dt;
  A(StateDef::VY, StateDef::AY) = dt;
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::computeQ(ProcessNoiseDiagMatrix& Q, const FloatType dt)
{
  // DWPA process, elements in Q define an acceleration increment, i.e. unit is equal to m/s**2
  Q = {static_cast<FloatType>(100.0 * dt * dt), static_cast<FloatType>(100.0 * dt * dt)};
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::computeG(ProcessNoiseMappingMatrix& G, const FloatType dt)
{
  const FloatType halfDeltaTimePow2 = static_cast<FloatType>(0.5) * dt * dt;

  G.setZeros();
  G(X, Q_AX) = halfDeltaTimePow2;
  G(VX, Q_AX) = dt;
  G(AX, Q_AX) = static_cast<FloatType>(1.0);
  G(Y, Q_AY) = halfDeltaTimePow2;
  G(VY, Q_AY) = dt;
  G(AY, Q_AY) = static_cast<FloatType>(1.0);
}

} // namespace motion
} // namespace tracking

#endif // EB85CC72_56D5_4D88_89CF_98C6580F5B61
