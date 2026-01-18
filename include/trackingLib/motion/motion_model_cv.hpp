#ifndef AE0B3445_6354_4B13_BF8F_59880FFFC470
#define AE0B3445_6354_4B13_BF8F_59880FFFC470

#include "motion/motion_model_cv.h"

#include "motion/generic_predict.hpp"     // IWYU pragma: keep
#include "motion/state_cov_converter.hpp" // IWYU pragma: keep
#include "motion/state_vec_converter.hpp" // IWYU pragma: keep

namespace tracking
{
namespace motion
{

template <typename CovarianceMatrixPolicy_>
MotionModelCV<CovarianceMatrixPolicy_>::MotionModelCV(const StateVec& vec, const StateCov& cov)
    : super_extended_mm_type{vec, cov}
    , super_generic_predict_type{}
{
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::predict(const FloatType         dt,
                                                     const KalmanFilterType& filter,
                                                     const EgoMotionType&    egoMotion)
{
  super_generic_predict_type::run(dt, filter, egoMotion);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::predict(const FloatType              dt,
                                                     const InformationFilterType& filter,
                                                     const EgoMotionType&         egoMotion)
{
  super_generic_predict_type::run(dt, filter, egoMotion);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::compensateEgoMotion(EgoMotionMappingMatrix& Ge,
                                                                 StateMatrix&            Go,
                                                                 const EgoMotionType&    egoMotion)
{
  FloatType& x  = this->operator[](StateDefCV::X);
  FloatType& y  = this->operator[](StateDefCV::Y);
  FloatType& vx = this->operator[](StateDefCV::VX);
  FloatType& vy = this->operator[](StateDefCV::VY);

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

  // translate and rotate position
  egoMotion.compensatePosition(x, y, x, y);
  // rotate velocity and acceleration
  egoMotion.compensateDirection(vx, vy, vx, vy);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::applyProcessModel(const FloatType dt)
{
  const StateVec& stateVec = this->getVec();

  this->operator[](StateDefCV::X) += dt * stateVec.at_unsafe(StateDefCV::VX);
  this->operator[](StateDefCV::Y) += dt * stateVec.at_unsafe(StateDefCV::VY);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::computeA(StateMatrix& A, const FloatType dt) const
{
  A.setIdentity();
  A.at_unsafe(StateDefCV::X, StateDefCV::VX) = dt;
  A.at_unsafe(StateDefCV::Y, StateDefCV::VY) = dt;
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::computeQ(ProcessNoiseDiagMatrix& Q, const FloatType /* dt */)
{
  // DWNA process, elements in Q define an acceleration, i.e. unit is equal to m/s**2
  Q.at_unsafe(Q_VX) = static_cast<FloatType>(10.0);
  Q.at_unsafe(Q_VY) = static_cast<FloatType>(10.0);
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::computeG(ProcessNoiseMappingMatrix& G, const FloatType dt)
{
  const FloatType halfDeltaTimePow2 = static_cast<FloatType>(0.5) * dt * dt;

  G.setZeros();
  G.at_unsafe(X, Q_VX)  = halfDeltaTimePow2;
  G.at_unsafe(VX, Q_VX) = dt;
  G.at_unsafe(Y, Q_VY)  = halfDeltaTimePow2;
  G.at_unsafe(VY, Q_VY) = dt;
}

template <typename CovarianceMatrixPolicy_>
void MotionModelCV<CovarianceMatrixPolicy_>::convertFrom(const MotionModelCA<CovarianceMatrixPolicy_>& other)
{
  using other_type = MotionModelCA<CovarianceMatrixPolicy_>;
  StateVecConverter<instance_type, other_type>::convertFrom(this->getVec(), other.getVec());
  StateCovConverter<instance_type, other_type>::convertFrom(this->getCov(), other.getCov());
}

} // namespace motion
} // namespace tracking

#endif // AE0B3445_6354_4B13_BF8F_59880FFFC470
