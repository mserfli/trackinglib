#ifndef D2E08F36_A9FC_4D2E_9737_D64D5B314B9E
#define D2E08F36_A9FC_4D2E_9737_D64D5B314B9E

#include "motion/motion_model_ca.h"

#include "motion/generic_predict.hpp"     // IWYU pragma: keep
#include "motion/state_cov_converter.hpp" // IWYU pragma: keep
#include "motion/state_vec_converter.hpp" // IWYU pragma: keep

namespace tracking
{
namespace motion
{

template <typename CovarianceMatrixPolicy_>
MotionModelCA<CovarianceMatrixPolicy_>::MotionModelCA(const StateVec& vec, const StateCov& cov)
    : super_extended_mm_type{vec, cov}
    , super_generic_predict_type{}
{
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::predict(const FloatType         dt,
                                                     const KalmanFilterType& filter,
                                                     const EgoMotionType&    egoMotion)
{
  super_generic_predict_type::run(dt, filter, egoMotion);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::predict(const FloatType              dt,
                                                     const InformationFilterType& filter,
                                                     const EgoMotionType&         egoMotion)
{
  super_generic_predict_type::run(dt, filter, egoMotion);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::compensateEgoMotion(EgoMotionMappingMatrix& Ge,
                                                                 StateMatrix&            Go,
                                                                 const EgoMotionType&    egoMotion)
{
  FloatType& x  = this->operator[](StateDefCA::X);
  FloatType& y  = this->operator[](StateDefCA::Y);
  FloatType& vx = this->operator[](StateDefCA::VX);
  FloatType& vy = this->operator[](StateDefCA::VY);
  FloatType& ax = this->operator[](StateDefCA::AX);
  FloatType& ay = this->operator[](StateDefCA::AY);

  const FloatType sinDeltaPsiEgo = egoMotion.getDisplacementCog().sinDeltaPsi;
  const FloatType cosDeltaPsiEgo = egoMotion.getDisplacementCog().cosDeltaPsi;
  const FloatType deltaXEgo      = egoMotion.getDisplacementCog().vec.at_unsafe(EgoMotionType::DS_X);
  const FloatType deltaYEgo      = egoMotion.getDisplacementCog().vec.at_unsafe(EgoMotionType::DS_Y);
  const FloatType distCog2Ego    = egoMotion.getGeometry().distCog2Ego;

  Go.setZeros();
  Go.at_unsafe(X, X)   = cosDeltaPsiEgo;
  Go.at_unsafe(X, Y)   = sinDeltaPsiEgo;
  Go.at_unsafe(Y, X)   = -sinDeltaPsiEgo;
  Go.at_unsafe(Y, Y)   = cosDeltaPsiEgo;
  Go.at_unsafe(VX, VX) = cosDeltaPsiEgo;
  Go.at_unsafe(VX, VY) = sinDeltaPsiEgo;
  Go.at_unsafe(VY, VX) = -sinDeltaPsiEgo;
  Go.at_unsafe(VY, VY) = cosDeltaPsiEgo;
  Go.at_unsafe(AX, AX) = cosDeltaPsiEgo;
  Go.at_unsafe(AX, AY) = sinDeltaPsiEgo;
  Go.at_unsafe(AY, AX) = -sinDeltaPsiEgo;
  Go.at_unsafe(AY, AY) = cosDeltaPsiEgo;

  const FloatType x0 = -deltaYEgo + y;
  const FloatType x1 = deltaXEgo - distCog2Ego - x;
  Ge.setZeros();
  Ge.at_unsafe(X, EgoMotionType::DS_X)    = -cosDeltaPsiEgo;
  Ge.at_unsafe(X, EgoMotionType::DS_Y)    = -sinDeltaPsiEgo;
  Ge.at_unsafe(X, EgoMotionType::DS_PSI)  = (x0 * cosDeltaPsiEgo) + (x1 * sinDeltaPsiEgo);
  Ge.at_unsafe(Y, EgoMotionType::DS_X)    = sinDeltaPsiEgo;
  Ge.at_unsafe(Y, EgoMotionType::DS_Y)    = cosDeltaPsiEgo;
  Ge.at_unsafe(Y, EgoMotionType::DS_PSI)  = -(x0 * sinDeltaPsiEgo) + (x1 * cosDeltaPsiEgo);
  Ge.at_unsafe(VX, EgoMotionType::DS_PSI) = -(vx * sinDeltaPsiEgo) + (vy * cosDeltaPsiEgo);
  Ge.at_unsafe(VY, EgoMotionType::DS_PSI) = -(vx * cosDeltaPsiEgo) - (vy * sinDeltaPsiEgo);
  Ge.at_unsafe(AX, EgoMotionType::DS_PSI) = -(ax * sinDeltaPsiEgo) + (ay * cosDeltaPsiEgo);
  Ge.at_unsafe(AY, EgoMotionType::DS_PSI) = -(ax * cosDeltaPsiEgo) - (ay * sinDeltaPsiEgo);

  // translate and rotate position
  egoMotion.compensatePosition(x, y, x, y);
  // rotate velocity and acceleration
  egoMotion.compensateDirection(vx, vy, vx, vy);
  egoMotion.compensateDirection(ax, ay, ax, ay);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::applyProcessModel(const FloatType dt)
{
  const StateVec& stateVec = this->getVec();
  const auto      halfDtSq = static_cast<FloatType>(0.5) * dt * dt;

  this->operator[](StateDefCA::X) += dt * stateVec.at_unsafe(StateDefCA::VX) + halfDtSq * stateVec.at_unsafe(StateDefCA::AX);
  this->operator[](StateDefCA::Y) += dt * stateVec.at_unsafe(StateDefCA::VY) + halfDtSq * stateVec.at_unsafe(StateDefCA::AY);
  this->operator[](StateDefCA::VX) += dt * stateVec.at_unsafe(StateDefCA::AX);
  this->operator[](StateDefCA::VY) += dt * stateVec.at_unsafe(StateDefCA::AY);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::computeA(StateMatrix& A, const FloatType dt) const
{
  const auto halfDtSq = static_cast<FloatType>(0.5) * dt * dt;

  A.setIdentity();
  A.at_unsafe(StateDefCA::X, StateDefCA::AX)  = halfDtSq;
  A.at_unsafe(StateDefCA::X, StateDefCA::VX)  = dt;
  A.at_unsafe(StateDefCA::VX, StateDefCA::AX) = dt;

  A.at_unsafe(StateDefCA::Y, StateDefCA::AY)  = halfDtSq;
  A.at_unsafe(StateDefCA::Y, StateDefCA::VY)  = dt;
  A.at_unsafe(StateDefCA::VY, StateDefCA::AY) = dt;
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::computeQ(ProcessNoiseDiagMatrix& Q, const FloatType dt)
{
  // DWPA process, elements in Q define an acceleration increment, i.e. unit is equal to m/s**2
  Q.at_unsafe(Q_AX) = static_cast<FloatType>(100.0 * dt * dt);
  Q.at_unsafe(Q_AY) = static_cast<FloatType>(100.0 * dt * dt);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::computeG(ProcessNoiseMappingMatrix& G, const FloatType dt)
{
  const FloatType halfDeltaTimePow2 = static_cast<FloatType>(0.5) * dt * dt;

  G.setZeros();
  G.at_unsafe(X, Q_AX)  = halfDeltaTimePow2;
  G.at_unsafe(VX, Q_AX) = dt;
  G.at_unsafe(AX, Q_AX) = static_cast<FloatType>(1.0);
  G.at_unsafe(Y, Q_AY)  = halfDeltaTimePow2;
  G.at_unsafe(VY, Q_AY) = dt;
  G.at_unsafe(AY, Q_AY) = static_cast<FloatType>(1.0);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCA<CovarianceMatrixPolicy_>::convertFrom(const MotionModelCV<CovarianceMatrixPolicy_>& other)
{
  using other_type = MotionModelCV<CovarianceMatrixPolicy_>;
  StateVecConverter<instance_type, other_type>::convertFrom(this->getVec(), other.getVec());
  StateCovConverter<instance_type, other_type>::convertFrom(this->getCov(), other.getCov());
}

} // namespace motion
} // namespace tracking

#endif // D2E08F36_A9FC_4D2E_9737_D64D5B314B9E
