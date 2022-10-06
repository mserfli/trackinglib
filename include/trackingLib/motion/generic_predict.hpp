#ifndef A5CB3020_67E9_40A3_8F41_9346328EADA3
#define A5CB3020_67E9_40A3_8F41_9346328EADA3

#include "motion/generic_predict.h"

namespace tracking
{
namespace motion
{
namespace generic
{

template <typename MotionModel, typename FloatType>
inline void Predict<MotionModel, FloatType, math::CovarianceMatrixFull>::run(const FloatType                        dt,
                                                                             const filter::KalmanFilter<FloatType>& filter,
                                                                             const env::EgoMotion<FloatType>&       egoMotion)
{
  static typename PredictCommon<MotionModel, FloatType>::Storage data{};
  PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

  auto& underlying = static_cast<MotionModel&>(*this);
  auto& P          = underlying.getCov();
  // TODO(matthias): use static allocation like http://blackforrest-embedded.de/2019/09/26/a-templated-static-allocator/
  // apply ego motion compensation on P
  P = (data.Go * P * data.Go.transpose()) + (data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose());

  filter.predictCovariance(P, data.A, data.G, data.Q);
}

template <typename MotionModel, typename FloatType>
inline void Predict<MotionModel, FloatType, math::CovarianceMatrixFull>::run(const FloatType                             dt,
                                                                             const filter::InformationFilter<FloatType>& filter,
                                                                             const env::EgoMotion<FloatType>& egoMotion)
{
  static typename PredictCommon<MotionModel, FloatType>::Storage data{};
  PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

  auto& underlying = static_cast<MotionModel&>(*this);
  auto& Y          = underlying.getCov();
  assert(Y.isInverse() && "covariance has to represent the inverse covariance");
#if 0    
    // TODO(matthias): ego motion compensation is quite complicated here, maybe neglect influence of Ge*Pe*Ge'
    // reconstruct P which might cause issues because P becomes extremly large
    static auto postP = cov.inverse();
    // apply ego motion compensation on P
    static auto compP = (data.Go * postP * data.Go.transpose())
                      + (data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose());
    // calc compensated Y
    Y = compP.inverse();
#else
  static auto invGo = data.Go.inverse();
  // calc compensated Y, neglecting Ge
  Y = invGo.transpose() * Y * invGo;
#endif
  filter.predictCovariance(Y, data.A, data.G, data.Q);
}

template <typename MotionModel, typename FloatType>
inline void Predict<MotionModel, FloatType, math::CovarianceMatrixFactored>::run(const FloatType                        dt,
                                                                                 const filter::KalmanFilter<FloatType>& filter,
                                                                                 const env::EgoMotion<FloatType>&       egoMotion)
{
  static typename PredictCommon<MotionModel, FloatType>::Storage data{};
  PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

  auto& underlying = static_cast<MotionModel&>(*this);
  auto& P          = underlying.getCov();

  static typename MotionModel::StateMatrix AGo = data.A * data.Go;

  // TODO(matthias): complete calculations of the factored predict

  // TODO(matthias): Optimization - provide a new BlockDiagonal matrix class to reduce operations on known zero elements
  static typename MotionModel::AugmentedProcessNoiseDiagMatrix Qstar; // [De 0; 0 Q]
  // TODO(matthias): Optimization - we could also have a vector of matrices to avoid constructing augmented matrices with copy
  // operations
  static typename MotionModel::AugmentedProcessNoiseMappingMatrix Gstar; // [A*Ge*Ue G]

  filter.predictCovariance(P, AGo, Gstar, Qstar);
}

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // A5CB3020_67E9_40A3_8F41_9346328EADA3
