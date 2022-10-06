#ifndef AE0B3445_6354_4B13_BF8F_59880FFFC470
#define AE0B3445_6354_4B13_BF8F_59880FFFC470

#include "motion/motion_model_cv.h"

#include "motion/state_cov_converter.hpp"
#include "motion/state_vec_converter.hpp"

namespace tracking
{
namespace motion
{

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::predict(const FloatType                        dt,
                                                             const filter::KalmanFilter<FloatType>& filter,
                                                             const env::EgoMotion<FloatType>&       egoMotion)
{
  super_generic_predict_type::run(dt, filter, egoMotion);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::compensateEgoMotion(EgoMotionMappingMatrix& Ge,
                                                                         StateMatrix&            Go,
                                                                         const EgoMotion&        egoMotion)
{
  FloatType& x  = this->operator[](StateDef::X);
  FloatType& y  = this->operator[](StateDef::Y);
  FloatType& vx = this->operator[](StateDef::VX);
  FloatType& vy = this->operator[](StateDef::VY);

  const FloatType sinDeltaPsiEgo = egoMotion.getDisplacementCog().sinDeltaPsi;
  const FloatType cosDeltaPsiEgo = egoMotion.getDisplacementCog().cosDeltaPsi;
  const FloatType deltaXEgo      = egoMotion.getDisplacementCog().vec[EgoMotion::DS_X];
  const FloatType deltaYEgo      = egoMotion.getDisplacementCog().vec[EgoMotion::DS_Y];
  const FloatType distCog2Ego    = egoMotion.getGeometry().distCog2Ego;

  Go.setZeros();
  Go(X, X)   = cosDeltaPsiEgo;
  Go(X, Y)   = sinDeltaPsiEgo;
  Go(Y, X)   = -sinDeltaPsiEgo;
  Go(Y, Y)   = cosDeltaPsiEgo;
  Go(VX, VX) = cosDeltaPsiEgo;
  Go(VX, VY) = sinDeltaPsiEgo;
  Go(VY, VX) = -sinDeltaPsiEgo;
  Go(VY, VY) = cosDeltaPsiEgo;

  const FloatType x0 = -deltaYEgo + y;
  const FloatType x1 = deltaXEgo - distCog2Ego - x;
  Ge.setZeros();
  Ge(X, EgoMotion::DS_X)    = -cosDeltaPsiEgo;
  Ge(X, EgoMotion::DS_Y)    = -sinDeltaPsiEgo;
  Ge(X, EgoMotion::DS_PSI)  = (x0 * cosDeltaPsiEgo) + (x1 * sinDeltaPsiEgo);
  Ge(Y, EgoMotion::DS_X)    = sinDeltaPsiEgo;
  Ge(Y, EgoMotion::DS_Y)    = cosDeltaPsiEgo;
  Ge(Y, EgoMotion::DS_PSI)  = -(x0 * sinDeltaPsiEgo) + (x1 * cosDeltaPsiEgo);
  Ge(VX, EgoMotion::DS_PSI) = -(vx * sinDeltaPsiEgo) + (vy * cosDeltaPsiEgo);
  Ge(VY, EgoMotion::DS_PSI) = -(vx * cosDeltaPsiEgo) - (vy * sinDeltaPsiEgo);

  // translate and rotate position
  egoMotion.compensatePosition(x, y, x, y);
  // rotate velocity and acceleration
  egoMotion.compensateDirection(vx, vy, vx, vy);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::applyProcessModel(const FloatType dt)
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
void MotionModelCV<CovarianceMatrixType, FloatType>::computeQ(ProcessNoiseDiagMatrix& Q, const FloatType /* dt */)
{
  // DWNA process, elements in Q define an acceleration, i.e. unit is equal to m/s**2
  Q = {static_cast<FloatType>(10.0), static_cast<FloatType>(10.0)};
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::computeG(ProcessNoiseMappingMatrix& G, const FloatType dt)
{
  const FloatType halfDeltaTimePow2 = static_cast<FloatType>(0.5) * dt * dt;

  G.setZeros();
  G(X, Q_VX)  = halfDeltaTimePow2;
  G(VX, Q_VX) = dt;
  G(Y, Q_VY)  = halfDeltaTimePow2;
  G(VY, Q_VY) = dt;
}

template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType, typename FloatType>
void MotionModelCV<CovarianceMatrixType, FloatType>::convertFrom(const MotionModelCA<CovarianceMatrixType, FloatType>& other)
{
  using other_type = MotionModelCA<CovarianceMatrixType, FloatType>;
  StateVecConverter<instance_type, other_type, FloatType, CovarianceMatrixType>::convertFrom(this->getVec(), other.getVec());
  StateCovConverter<instance_type, other_type, FloatType>::convertFrom(this->getCov(), other.getCov());
}

} // namespace motion
} // namespace tracking

#endif // AE0B3445_6354_4B13_BF8F_59880FFFC470
