#ifndef A7B8C9D0_2E3F_4A4B_F5C6_7D8E9F0A1B2C
#define A7B8C9D0_2E3F_4A4B_F5C6_7D8E9F0A1B2C

#include "motion/generic_update.h"

#include "filter/information_filter.hpp" // IWYU pragma: keep
#include "filter/kalman_filter.hpp"      // IWYU pragma: keep
#include "math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/matrix.hpp"                     // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"              // IWYU pragma: keep
#include "math/linalg/vector.hpp"                     // IWYU pragma: keep
#include <type_traits>
#include <utility>

namespace tracking
{
namespace motion
{
namespace generic
{

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename UpdateMode_, typename... ObservationModels_>
inline void Update<MotionModel_, CovarianceMatrixPolicy_>::run(MotionModel_&           motionModel,
                                                               const KalmanFilterType& filter,
                                                               const EgoMotionType&    egoMotion,
                                                               const ObservationModels_&... observationModels)
{
  static_assert(sizeof...(ObservationModels_) > 0, "at least one observation model is required");
  static_assert((std::is_same_v<typename ObservationModels_::StateDef, typename MotionModel_::StateDef> && ...),
                "all observation models must observe the motion model's StateDef");
  static_assert((std::is_same_v<typename ObservationModels_::CovarianceMatrixPolicy, CovarianceMatrixPolicy_> && ...),
                "all observation models must use the motion model's covariance matrix policy");

  constexpr sint32 DimX      = MotionModel_::StateDef::NUM_STATE_VARIABLES;
  constexpr sint32 TotalDimZ = (ObservationModels_::DimZ + ...);
  static_assert(TotalDimZ > 0, "total measurement dimension must be positive");

  // Access to MotionModel internals
  auto& x = motionModel.getVecForInternalUse();
  auto& P = motionModel.getCovForInternalUse();

  // stack innovation, Jacobian and (block-diagonal) covariance of all observation models
  math::Vector<value_type, TotalDimZ>       innovation{};
  math::Matrix<value_type, TotalDimZ, DimX> H{};
  math::SquareMatrix<value_type, TotalDimZ> Rfull{};
  stackObservations<TotalDimZ, DimX, 0>(innovation, H, Rfull, x, egoMotion, observationModels...);

  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    // keep the full-fidelity R (incl. correlations): the sequential update decorrelates internally
    Rfull.symmetrize();
    const auto R = math::conversions::CovarianceMatrixFactoredFromCovarianceMatrixFull<value_type, TotalDimZ>(
        math::CovarianceMatrixFull<value_type, TotalDimZ>{std::move(Rfull)});
    if (!R.has_value())
    {
      return; // defensive: stacked R could not be factorized, skip the update
    }
    filter.template updateState<UpdateMode_>(x, P, innovation, H, R.value());
  }
  else
  {
    Rfull.symmetrize();
    const math::CovarianceMatrixFull<value_type, TotalDimZ> R{std::move(Rfull)};
    filter.template updateState<UpdateMode_>(x, P, innovation, H, R);
  }
}

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename UpdateMode_, typename... ObservationModels_>
inline void Update<MotionModel_, CovarianceMatrixPolicy_>::run(MotionModel_&                motionModel,
                                                               const InformationFilterType& filter,
                                                               const EgoMotionType&         egoMotion,
                                                               const ObservationModels_&... observationModels)
{
  static_assert(sizeof...(ObservationModels_) > 0, "at least one observation model is required");
  static_assert((std::is_same_v<typename ObservationModels_::StateDef, typename MotionModel_::StateDef> && ...),
                "all observation models must observe the motion model's StateDef");
  static_assert((std::is_same_v<typename ObservationModels_::CovarianceMatrixPolicy, CovarianceMatrixPolicy_> && ...),
                "all observation models must use the motion model's covariance matrix policy");

  constexpr sint32 DimX      = MotionModel_::StateDef::NUM_STATE_VARIABLES;
  constexpr sint32 TotalDimZ = (ObservationModels_::DimZ + ...);
  static_assert(TotalDimZ > 0, "total measurement dimension must be positive");

  // Step 1: Transform from information space to state space to evaluate the observation models
  motionModel.convertStateVecIntoStateSpace();
  const math::Vector<value_type, DimX> xState{motionModel.getVecForInternalUse()};

  // Step 2: stack innovation, Jacobian and (block-diagonal) covariance of all observation models
  math::Vector<value_type, TotalDimZ>       innovation{};
  math::Matrix<value_type, TotalDimZ, DimX> H{};
  math::SquareMatrix<value_type, TotalDimZ> Rfull{};
  stackObservations<TotalDimZ, DimX, 0>(innovation, H, Rfull, xState, egoMotion, observationModels...);

  // Step 3: effective measurement of the linearized observation model: z_eff = nu + H*x
  const math::Vector<value_type, TotalDimZ> zEff{innovation + (H * xState)};

  // Step 4: Transform back to information space using the PRIOR information matrix, then apply
  // the purely additive information update
  motionModel.convertStateVecIntoInformationSpace();
  auto& y = motionModel.getVecForInternalUse();
  auto& Y = motionModel.getCovForInternalUse();

  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    // keep the full-fidelity R (incl. correlations): the sequential update decorrelates internally
    Rfull.symmetrize();
    const auto R = math::conversions::CovarianceMatrixFactoredFromCovarianceMatrixFull<value_type, TotalDimZ>(
        math::CovarianceMatrixFull<value_type, TotalDimZ>{std::move(Rfull)});
    if (!R.has_value())
    {
      return; // defensive: stacked R could not be factorized, skip the update
    }
    filter.template updateState<UpdateMode_>(y, Y, zEff, H, R.value());
  }
  else
  {
    Rfull.symmetrize();
    const math::CovarianceMatrixFull<value_type, TotalDimZ> R{std::move(Rfull)};
    filter.template updateState<UpdateMode_>(y, Y, zEff, H, R);
  }
}

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <sint32 TotalDimZ_, sint32 DimX_, sint32 Offset_>
inline void Update<MotionModel_, CovarianceMatrixPolicy_>::stackObservations(math::Vector<value_type, TotalDimZ_>& /*innovation*/,
                                                                             math::Matrix<value_type, TotalDimZ_, DimX_>& /*H*/,
                                                                             math::SquareMatrix<value_type, TotalDimZ_>& /*R*/,
                                                                             const math::Vector<value_type, DimX_>& /*state*/,
                                                                             const EgoMotionType& /*egoMotion*/)
{
  // recursion end: all observation models are stacked
  static_assert(Offset_ == TotalDimZ_, "stacked observation dimensions do not sum up to TotalDimZ");
}

template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <sint32 TotalDimZ_, sint32 DimX_, sint32 Offset_, typename First_, typename... Rest_>
inline void Update<MotionModel_, CovarianceMatrixPolicy_>::stackObservations(math::Vector<value_type, TotalDimZ_>& innovation,
                                                                             math::Matrix<value_type, TotalDimZ_, DimX_>& H,
                                                                             math::SquareMatrix<value_type, TotalDimZ_>&  R,
                                                                             const math::Vector<value_type, DimX_>&       state,
                                                                             const EgoMotionType& egoMotion,
                                                                             const First_&        first,
                                                                             const Rest_&... rest)
{
  constexpr sint32 DimZ = First_::DimZ;
  static_assert((Offset_ + DimZ) <= TotalDimZ_, "stacked observation dimensions exceed TotalDimZ");

  // Jacobian block
  typename First_::JacobianMatrix Hi{};
  first.computeJacobian(Hi, state, egoMotion);
  H.template setBlock<DimZ, DimX_, DimZ, DimX_, 0, 0, true, Offset_, 0>(Hi);

  // innovation block (innovation handling, e.g. angle wrapping, is resolved on the concrete model)
  const auto predicted = first.predictMeasurement(state, egoMotion);
  const auto nu        = first.computeInnovation(first.getVec(), predicted);
  for (sint32 i = 0; i < DimZ; ++i)
  {
    innovation.at_unsafe(Offset_ + i) = nu.at_unsafe(i);
  }

  // measurement covariance block (composed to full form; models are assumed uncorrelated)
  const auto& Ri = first.getCov()();
  R.template setBlock<DimZ, DimZ, DimZ, DimZ, 0, 0, true, Offset_, Offset_>(Ri);

  stackObservations<TotalDimZ_, DimX_, Offset_ + DimZ>(innovation, H, R, state, egoMotion, rest...);
}

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // A7B8C9D0_2E3F_4A4B_F5C6_7D8E9F0A1B2C
