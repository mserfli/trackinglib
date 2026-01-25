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
  static typename BasePredictCommon::Storage data{};
  BasePredictCommon::run(data, dt, egoMotion);

  auto& underlying = static_cast<MotionModel_&>(*this);
  auto& P          = underlying.getCovForInternalUse();

  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    if (egoMotion.getDisplacementCog().vec.isZeros())
    {
      filter.predictCovariance(P, data.AGo, data.G, data.Q);
    }
    else
    {
      filter.predictCovariance(P, data.AGo, data.Gstar, data.Qstar);
    }
  }
  else
  {
    // apply ego motion compensation on P
    P.apaT(data.Go);
    P += data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose();
    P.symmetrize();

    // apply time update
    filter.predictCovariance(P, data.A, data.G, data.Q);
  }
}

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
inline void Predict<MotionModel_, CovarianceMatrixPolicy_>::run(const value_type             dt,
                                                                const InformationFilterType& filter,
                                                                const EgoMotionType&         egoMotion)
{
  static typename BasePredictCommon::Storage data{};
  BasePredictCommon::run(data, dt, egoMotion);

  auto& underlying = static_cast<MotionModel_&>(*this);
  auto& Y          = underlying.getCovForInternalUse();

  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    if (egoMotion.getDisplacementCog().vec.isZeros())
    {
      filter.predictCovariance(Y, data.AGo, data.G, data.Q);
    }
    else
    {
      filter.predictCovariance(Y, data.AGo, data.Gstar, data.Qstar);
    }
  }
  else
  {
    if (egoMotion.getDisplacementCog().vec.isZeros()) // TODO(matthias): is this really correct, Yes: because we are checking if
                                                      // the displacement vector is all zeros without tolerances!
    {
      // filter prediction step with zero ego motion
      filter.predictCovariance(Y, data.A, data.G, data.Q);
    }
    else
    {
      using EgoMotionCov = typename EgoMotionType::DisplacementCov;
      using StateCov     = typename MotionModel_::StateCov;
      using StateMatrix  = typename MotionModel_::StateMatrix;
      // Y = inv(Go*P*Go.T + Ge*Pe*Ge.T) with Po=inv(Y) can be solved directly in information space by
      // applying two steps
      // 1. Handle the State Transition
      //    Ys = inv(Go*P*Go.T) = inv(Go).T * Y * inv(Go)
      // 2. Handle the Additive Noise Q=Ge*Pe*Ge.T
      //    Y = inv(Ps + Q) applying the Woodbury Matrix Identity gives
      //    Y = Ys - Ys*Ge * inv( inv(Pe) + Ge.T*Ys*Ge ) * Ge.T*Ys

      // step 1: Handle the State Transition (using linear solver)
      StateMatrix Z_T = data.Go.transpose().qrSolve(Y);                                    // solve Go.T * Z_transpose = Y
      auto        Ytr = StateCov{std::move(data.Go.transpose().qrSolve(Z_T.transpose()))}; // solve Go.T * Y_tr = Z
      Ytr.symmetrize();

      // step 2: Handle the Additive Noise to be applied on the transformed Ytr (different to the one-step factored solution)!!!
      // apply Woodbury Matrix Identity
      // (Y^-1 + Ge Pe Ge')^-1 = Y - Y Ge (Pe^-1 + Ge' Y Ge)^-1 Ge' Y
      // note: if any of the inverses fails, we skip the ego motion compensation step
      const auto invPe = egoMotion.getDisplacementCog().cov.inverse();
      if (invPe.has_value())
      {
        // Compute Y Ge
        auto YGe =
            math::Matrix<value_type, MotionModel_::NUM_STATE_VARIABLES, EgoMotionType::DS_NUM_VARIABLES, true>{Ytr * data.Ge};

        // Compute Ge' Y
        auto GeTY = math::Matrix<value_type, EgoMotionType::DS_NUM_VARIABLES, MotionModel_::NUM_STATE_VARIABLES, false>{
            data.Ge.transpose() * Ytr};

        // Compute Ge' Y Ge
        auto GeTYGe = math::SquareMatrix<value_type, EgoMotionType::DS_NUM_VARIABLES, false>{GeTY * data.Ge};

        // Compute M = (Pe^-1 + Ge' Y Ge)^-1
        auto M = EgoMotionCov{typename EgoMotionCov::BaseSquareMatrix{invPe.value() + std::move(GeTYGe)}};
        M.symmetrize();
        const auto invM = M.inverse();
        if (invM.has_value())
        {
          const auto mat = std::move(YGe) * std::move(invM.value()) * std::move(GeTY);
          // Y^-1 + Ge Pe Ge')^-1 = Y - Y Ge (Pe^-1 + Ge' Y Ge)^-1 Ge' Y
          Ytr -= mat;
          Ytr.symmetrize();

          // final filter prediction step with compensated Ytr
          filter.predictCovariance(Ytr, data.A, data.G, data.Q);
          Y = std::move(Ytr);
        }
      }
    }
  }
}

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // A5CB3020_67E9_40A3_8F41_9346328EADA3
