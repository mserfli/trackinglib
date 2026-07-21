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
  // Access to MotionModel internals
  auto& underlying = static_cast<MotionModel_&>(*this);
  auto& P          = underlying.getCovForInternalUse();

  static typename BasePredictCommon::Storage data{};
  BasePredictCommon::run(data, dt, egoMotion);

  // Covariance prediction
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
  // Access to MotionModel internals
  auto& underlying = static_cast<MotionModel_&>(*this);
  auto& Y          = underlying.getCovForInternalUse();

  // prepare covariance prediction based on egomotion compensated state
  static typename BasePredictCommon::Storage data{};

  // Step 1: Transform from information space to state space for state prediction
  underlying.convertStateVecIntoStateSpace();

  // Step 2: Run state prediction in state space (existing code)
  BasePredictCommon::run(data, dt, egoMotion);

  // Step 3: Covariance prediction (Y_k → Y_k+1)
  // NOTE: This MUST happen BEFORE convertStateVecIntoInformationSpace() because
  // the transformation needs the NEW information matrix Y_k+1
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
      using StateCov    = typename MotionModel_::StateCov;
      using StateMatrix = typename MotionModel_::StateMatrix;
      // Y = inv(Go*P*Go.T + Ge*Pe*Ge.T) with Po=inv(Y) can be solved directly in information space by
      // applying two steps
      // 1. Handle the State Transition
      //    Ys = inv(Go*P*Go.T) = inv(Go).T * Y * inv(Go)
      // 2. Handle the Additive Noise Q=Ge*Pe*Ge.T
      //    Y = inv(Ps + Q), solved via H*Y = Ys with H = I + Ys*Q using QR (mirrors
      //    InformationFilter::predictCovariance's own non-factored time-update branch for G*Q*G',
      //    see information_filter.hpp) instead of inverting Pe via the Woodbury identity: Pe is
      //    structurally rank-deficient whenever the ego-motion output dimension exceeds its
      //    independent noise sources, so inverting it directly is not robust.

      // step 1: Handle the State Transition (using linear solver)
      StateMatrix Z_T = data.Go.transpose().qrSolve(Y);                                    // solve Go.T * Z_transpose = Y
      auto        Ytr = StateCov{std::move(data.Go.transpose().qrSolve(Z_T.transpose()))}; // solve Go.T * Y_tr = Z
      Ytr.symmetrize();

      // step 2: Handle the Additive Noise Ge*Pe*Ge.T on the transformed Ytr (never inverts Pe)
      auto       Mtr = Ytr; // state-transitioned copy consumed by the QR solve below
      const auto H    = math::SquareMatrix<value_type, MotionModel_::NUM_STATE_VARIABLES>(
          StateMatrix::Identity() + Mtr * (data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose()));
      math::SquareMatrix compensated = H.qrSolve(std::move(Mtr));
      compensated.symmetrize();

      // prevent destroying the Information matrix, e.g. removing information from a zero Y matrix (no information)
      if (compensated.isPositiveSemiDefinite())
      {
        Ytr = StateCov{std::move(compensated)};

        // final filter prediction step with compensated Ytr
        filter.predictCovariance(Ytr, data.A, data.G, data.Q);

        // prevent destroying the Information matrix, e.g. removing information from a zero Y matrix (no information)
        if (Ytr.isPositiveSemiDefinite())
        {
          Y = std::move(Ytr);
        }
      }
    }
  }

  // Step 4: Transform state to information vector using the NEW information matrix Y_k+1
  underlying.convertStateVecIntoInformationSpace();
}

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // A5CB3020_67E9_40A3_8F41_9346328EADA3
