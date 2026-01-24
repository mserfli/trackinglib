#ifndef A5CB3020_67E9_40A3_8F41_9346328EADA3
#define A5CB3020_67E9_40A3_8F41_9346328EADA3

#include "motion/generic_predict.h"

#include "filter/information_filter.hpp"     // IWYU pragma: keep
#include "filter/kalman_filter.hpp"          // IWYU pragma: keep
#include "math/linalg/square_matrix.h"       // IWYU pragma: keep
#include "motion/generic_predict_common.hpp" // IWYU pragma: keep

namespace tracking
{
namespace motion
{
namespace generic
{

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
inline void Predict<MotionModel_, CovarianceMatrixPolicy_>::run(const value_type        dt,
                                                                const KalmanFilterType& filter,
                                                                const EgoMotionType&    egoMotion)
{
  static typename super_predict_common_type::Storage data{};
  super_predict_common_type::run(data, dt, egoMotion);

  auto& underlying = static_cast<MotionModel_&>(*this);
  auto& P          = underlying.getCovForInternalUse();

  using CovarianceMatrixPolicy = typename MotionModel_::instance_trait::CovarianceMatrixPolicy;
  if constexpr (CovarianceMatrixPolicy::is_factored)
  {
    static auto AGo = typename MotionModel_::StateMatrix{data.A * data.Go};

    // TODO(matthias): complete calculations of the factored predict

    // TODO(matthias): Optimization - provide a new BlockDiagonal matrix class to reduce operations on known zero elements
    static typename MotionModel_::AugmentedProcessNoiseDiagMatrix Qstar; // [De 0; 0 Q]
    // TODO(matthias): Optimization - we could also have a vector of matrixes to avoid constructing augmented matrixes with copy
    // operations
    static typename MotionModel_::AugmentedProcessNoiseMappingMatrix Gstar; // [A*Ge*Ue G]

    filter.predictCovariance(P, AGo, data.G, data.Q);
  }
  else
  {
    //  apply ego motion compensation on P
    P = typename MotionModel_::StateCov(typename MotionModel_::StateCov::SquareMatrix{
        (data.Go * P * data.Go.transpose()) + (data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose())});

    filter.predictCovariance(P, data.A, data.G, data.Q);
  }
}

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
inline void Predict<MotionModel_, CovarianceMatrixPolicy_>::run(const value_type             dt,
                                                                const InformationFilterType& filter,
                                                                const EgoMotionType&         egoMotion)
{
  static typename super_predict_common_type::Storage data{};
  super_predict_common_type::run(data, dt, egoMotion);

  auto& underlying = static_cast<MotionModel_&>(*this);
  auto& Y          = underlying.getCovForInternalUse();

  using CovarianceMatrixPolicy = typename MotionModel_::instance_trait::CovarianceMatrixPolicy;
  if constexpr (CovarianceMatrixPolicy::is_factored)
  {
    static auto AGo = typename MotionModel_::StateMatrix{data.A * data.Go};

    // TODO(matthias): complete calculations of the factored predict

    // TODO(matthias): Optimization - provide a new BlockDiagonal matrix class to reduce operations on known zero elements
    static typename MotionModel_::AugmentedProcessNoiseDiagMatrix Qstar; // [De 0; 0 Q]
    // TODO(matthias): Optimization - we could also have a vector of matrixes to avoid constructing augmented matrixes with copy
    // operations
    static typename MotionModel_::AugmentedProcessNoiseMappingMatrix Gstar; // [A*Ge*Ue G]

    // filter.predictCovariance(P, AGo, Gstar, Qstar);
    filter.predictCovariance(Y, AGo, data.G, data.Q);
  }
  else
  {
#if 0 // TODO(matthias): ego motion compensation is quite complicated here, maybe neglect influence of Ge*Pe*Ge'
    // reconstruct P which might cause issues because P becomes extremly large
    static auto postP = cov.inverse();
    // apply ego motion compensation on P
    static auto compP = (data.Go * postP * data.Go.transpose())
                      + (data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose());
    // calc compensated Y
    Y = compP.inverse();
    filter.predictCovariance(Y, data.A, data.G, data.Q);
#else
    // we neglect the uncertainty of Ge*De*Ge'
    filter.predictCovariance(Y, typename MotionModel_::StateMatrix(data.A * data.Go), data.G, data.Q);
#endif
  }
}

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // A5CB3020_67E9_40A3_8F41_9346328EADA3
