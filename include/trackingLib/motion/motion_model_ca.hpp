#ifndef D2E08F36_A9FC_4D2E_9737_D64D5B314B9E
#define D2E08F36_A9FC_4D2E_9737_D64D5B314B9E

#include "motion/motion_model_ca.h"

#include "motion/state_cov_converter.hpp"
#include "motion/state_vec_converter.hpp"

namespace tracking
{
namespace motion
{

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
MotionModelCA<CovarianceMatrixType, FloatType>::MotionModelCA(const StateVec& vec, const StateCov& cov)
    : super_extended_mm_type{vec, cov}
    , super_generic_predict_type{}
{
}

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
  FloatType& x  = this->operator[](StateDef::X);
  FloatType& y  = this->operator[](StateDef::Y);
  FloatType& vx = this->operator[](StateDef::VX);
  FloatType& vy = this->operator[](StateDef::VY);
  FloatType& ax = this->operator[](StateDef::AX);
  FloatType& ay = this->operator[](StateDef::AY);

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
  Go(AX, AX) = cosDeltaPsiEgo;
  Go(AX, AY) = sinDeltaPsiEgo;
  Go(AY, AX) = -sinDeltaPsiEgo;
  Go(AY, AY) = cosDeltaPsiEgo;

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
  A(StateDef::X, StateDef::AX)  = halfDtSq;
  A(StateDef::X, StateDef::VX)  = dt;
  A(StateDef::VX, StateDef::AX) = dt;

  A(StateDef::Y, StateDef::AY)  = halfDtSq;
  A(StateDef::Y, StateDef::VY)  = dt;
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
  G(X, Q_AX)  = halfDeltaTimePow2;
  G(VX, Q_AX) = dt;
  G(AX, Q_AX) = static_cast<FloatType>(1.0);
  G(Y, Q_AY)  = halfDeltaTimePow2;
  G(VY, Q_AY) = dt;
  G(AY, Q_AY) = static_cast<FloatType>(1.0);
}

template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::convertFrom(const MotionModelCV<CovarianceMatrixType, FloatType>& other)
{
  using other_type = MotionModelCV<CovarianceMatrixType, FloatType>;
  StateVecConverter<instance_type, other_type, FloatType, CovarianceMatrixType>::convertFrom(this->getVec(), other.getVec());
  StateCovConverter<instance_type, other_type, FloatType>::convertFrom(this->getCov(), other.getCov());
}

} // namespace motion
} // namespace tracking

#endif // D2E08F36_A9FC_4D2E_9737_D64D5B314B9E
