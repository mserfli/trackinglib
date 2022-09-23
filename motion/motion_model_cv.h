#ifndef motion_model_cv_h
#define motion_model_cv_h

#include "base/covariance_matrix_factored.h"
#include "base/covariance_matrix_full.h"
#include "base/matrix.h"
#include "env/ego_motion.h"
#include "motion/generic_predict.h"
#include "motion/imotion_model.h"

namespace tracking
{
namespace motion
{

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
class MotionModelCV
    : public ExtendedMotionModel<MotionModelCV<CovarianceMatrixType, FloatType>, CovarianceMatrixType, FloatType, 4>
    , public generic::Predict<MotionModelCV<CovarianceMatrixType, FloatType>, FloatType, CovarianceMatrixType>
{
public:
  enum NoiseDef
  {
    Q_VX = 0,
    Q_VY,
    NUM_PROC_NOISE_VARIABLES
  };

  enum StateDef
  {
    X = 0,
    VX,
    Y,
    VY,
    NUM_STATE_VARIABLES
  };

  using SuperGenericPredictor =
      generic::Predict<MotionModelCV<CovarianceMatrixType, FloatType>, FloatType, CovarianceMatrixType>;
  using SuperExtendedMotionModel =
      ExtendedMotionModel<MotionModelCV<CovarianceMatrixType, FloatType>, CovarianceMatrixType, FloatType, 4>;
  using StateVec = typename SuperExtendedMotionModel::StateVec;
  using StateCov = typename SuperExtendedMotionModel::StateCov;

  using StateMatrix = base::SquareMatrix<FloatType, NUM_STATE_VARIABLES>;
  using ProcessNoiseDiagMatrix = base::DiagonalMatrix<FloatType, NUM_PROC_NOISE_VARIABLES>;
  using ProcessNoiseMappingMatrix = base::Matrix<FloatType, NUM_STATE_VARIABLES, NUM_PROC_NOISE_VARIABLES>;

  using EgoMotion = env::EgoMotion<FloatType>;
  using EgoMotionMappingMatrix = base::Matrix<FloatType, NUM_STATE_VARIABLES, EgoMotion::DS_NUM_VARIABLES>;

  static constexpr sint32 NUM_AUG_PROC_NOISE_VARIABLES = NUM_PROC_NOISE_VARIABLES + EgoMotion::DS_NUM_VARIABLES;
  using AugmentedProcessNoiseDiagMatrix = base::DiagonalMatrix<FloatType, NUM_AUG_PROC_NOISE_VARIABLES>;
  using AugmentedProcessNoiseMappingMatrix = base::Matrix<FloatType, NUM_STATE_VARIABLES, NUM_AUG_PROC_NOISE_VARIABLES>;

  MotionModelCV() = default;
  MotionModelCV(const MotionModelCV<CovarianceMatrixType, FloatType>&) = default;
  MotionModelCV(MotionModelCV<CovarianceMatrixType, FloatType>&&) noexcept = default;
  auto operator=(const MotionModelCV<CovarianceMatrixType, FloatType>&)
      -> MotionModelCV<CovarianceMatrixType, FloatType>& = default;
  auto operator=(MotionModelCV<CovarianceMatrixType, FloatType>&&) noexcept
      -> MotionModelCV<CovarianceMatrixType, FloatType>& = default;
  ~MotionModelCV() = default;


  auto getVx() const -> FloatType final { return this->getVec()[StateDef::VX]; }
  // ... all the other virtual functions

  void predict(const FloatType                        dt,
               const filter::KalmanFilter<FloatType>& filter,
               const env::EgoMotion<FloatType>&       egoMotion) final
  {
    SuperGenericPredictor::run(dt, filter, egoMotion);
  }

  void        applyDynamicalModel(const FloatType dt);
  void        computeA(StateMatrix& A, const FloatType dt) const;
  void        compensateEgoMotion(EgoMotionMappingMatrix& Ge, StateMatrix& Go, const EgoMotion& egoMotion);
  static void computeQ(ProcessNoiseDiagMatrix& Q, const FloatType dt);
  static void computeG(ProcessNoiseMappingMatrix& G, const FloatType dt);
};

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::applyDynamicalModel(const FloatType dt)
{
  const StateVec& stateVec = this->getVec();

  this->operator[](StateDef::X) += dt * stateVec[StateDef::VX];
  this->operator[](StateDef::Y) += dt * stateVec[StateDef::VY];
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::computeA(StateMatrix& A, const FloatType dt) const
{
  A.setIdentity();
  A(StateDef::X, StateDef::VX) = dt;
  A(StateDef::Y, StateDef::VY) = dt;
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::compensateEgoMotion(EgoMotionMappingMatrix& Ge, StateMatrix& Go, const EgoMotion& egoMotion)
{
  FloatType& x = this->operator[](StateDef::X);
  FloatType& y = this->operator[](StateDef::Y);
  FloatType& vx = this->operator[](StateDef::VX);
  FloatType& vy = this->operator[](StateDef::VY);

  const FloatType& sinDeltaPsiEgo = egoMotion.getDisplacementCog().sinDeltaPsi;
  const FloatType  cosDeltaPsiEgo = egoMotion.getDisplacementCog().cosDeltaPsi;
  const FloatType  deltaXEgo = egoMotion.getDisplacementCog().vec[EgoMotion::DS_X];
  const FloatType  deltaYEgo = egoMotion.getDisplacementCog().vec[EgoMotion::DS_Y];
  const FloatType  distCog2Ego = egoMotion.getGeometry().distCog2Ego;

  Go.setZero();
  Go(X, X) = cosDeltaPsiEgo;
  Go(X, Y) = sinDeltaPsiEgo;
  Go(Y, X) = -sinDeltaPsiEgo;
  Go(Y, Y) = cosDeltaPsiEgo;
  Go(VX, VX) = cosDeltaPsiEgo;
  Go(VX, VY) = sinDeltaPsiEgo;
  Go(VY, VX) = -sinDeltaPsiEgo;
  Go(VY, VY) = cosDeltaPsiEgo;

  const FloatType x0 = -deltaYEgo + y;
  const FloatType x1 = deltaXEgo - distCog2Ego - x;
  Ge.setZero();
  Ge(X, EgoMotion::DS_X) = -cosDeltaPsiEgo;
  Ge(X, EgoMotion::DS_Y) = -sinDeltaPsiEgo;
  Ge(X, EgoMotion::DS_PSI) = (x0 * cosDeltaPsiEgo) + (x1 * sinDeltaPsiEgo);
  Ge(Y, EgoMotion::DS_X) = sinDeltaPsiEgo;
  Ge(Y, EgoMotion::DS_Y) = cosDeltaPsiEgo;
  Ge(Y, EgoMotion::DS_PSI) = -(x0 * sinDeltaPsiEgo) + (x1 * cosDeltaPsiEgo);
  Ge(VX, EgoMotion::DS_PSI) = -(vx * sinDeltaPsiEgo) + (vy * cosDeltaPsiEgo);
  Ge(VY, EgoMotion::DS_PSI) = -(vx * cosDeltaPsiEgo) - (vy * sinDeltaPsiEgo);

  // translate and rotate position
  egoMotion.compensatePosition(x, y, x, y);
  // rotate velocity and acceleration
  egoMotion.compensateDirection(vx, vy, vx, vy);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::computeQ(ProcessNoiseDiagMatrix& Q, const FloatType /* dt */)
{
  Q.diagonal().array()[Q_VX] = static_cast<FloatType>(10.0);
  Q.diagonal().array()[Q_VY] = static_cast<FloatType>(10.0);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::computeG(ProcessNoiseMappingMatrix& G, const FloatType dt)
{
  const FloatType halfDeltaTimePow2 = static_cast<FloatType>(0.5) * dt * dt;

  G.setZero();
  G(X, Q_VX) = halfDeltaTimePow2;
  G(VX, Q_VX) = dt;
  G(Y, Q_VY) = halfDeltaTimePow2;
  G(VY, Q_VY) = dt;
}

} // namespace motion
} // namespace tracking

#endif /* motion_model_cv_h */
