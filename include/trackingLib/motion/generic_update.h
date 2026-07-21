#ifndef F6A7B8C9_1D2E_4F3A_E4B5_6C7D8E9F0A1B
#define F6A7B8C9_1D2E_4F3A_E4B5_6C7D8E9F0A1B

#include "base/first_include.h" // IWYU pragma: keep
#include "env/ego_motion.h"
#include "filter/information_filter.h"
#include "filter/kalman_filter.h"
#include "filter/update_mode.h"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace motion
{
namespace generic
{

/// \brief Class to extend any motion model with a generic measurement update functionality
///
/// Mirrors generic::Predict for the measurement update path. The variadic run() overloads accept
/// one or more observation models: a single model performs a plain measurement update, multiple
/// models are composed by stacking their measurement vectors, Jacobians, and covariances into
/// one joint update (compile-time sized, TotalDimZ = sum of all model dimensions).
///
/// The update mode is selected at compile time (see filter::update_mode):
/// - Block:      one TotalDimZ-sized update (full covariance policy only)
/// - Sequential: scalar row-by-row updates (required for the factored policy); a correlated R
///               is decorrelated internally via its UDU factorization
///
/// Unlike generic::Predict (generic_predict.h), this class is NOT mixed into the motion model via
/// CRTP inheritance, and all of its methods are static as a result. ExtendedMotionModel's
/// update() (imotion_model.h) is itself a template method (variadic ObservationModels_..., plus an
/// UpdateMode_ tag) that explicitly calls generic::Update<...>::run(..., static_cast<MotionModel_&>
/// (*this), observationModels...), passing the motion model in as an ordinary parameter rather than
/// reaching it through an inherited `this`.
///
/// This is not an arbitrary style choice, nor a stylistic drift from Predict: it follows from a
/// hard C++ constraint. Predict is CRTP-inherited because IMotionModel::predict() is declared
/// virtual with a fixed, non-template signature, and only an actual base class can be overridden
/// that way. Update's run(), however, is inherently a template (it must vary per call by which and
/// how many observation models are passed) — and C++ does not allow virtual template member
/// functions. So update() can never be part of that same virtual-dispatch contract regardless of
/// how Update is implemented, which removes the only reason Predict needed inheritance in the
/// first place. With no virtual-interface pressure, implementing Update as a stateless static
/// template utility — invoked by qualified name, motion model passed in explicitly — avoids adding
/// an inheritance relationship that would buy nothing: the CRTP cast would still be needed, just
/// moved from the call site into a would-be non-static method body.
///
/// \tparam MotionModel_             The underlying MotionModel
/// \tparam CovarianceMatrixPolicy_  Policy type that defines the covariance matrix implementation
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class Update
{
public:
  using value_type            = typename CovarianceMatrixPolicy_::value_type;
  using KalmanFilterType      = filter::KalmanFilter<CovarianceMatrixPolicy_>;
  using InformationFilterType = filter::InformationFilter<CovarianceMatrixPolicy_>;
  using DefaultUpdateMode     = filter::update_mode::Default<CovarianceMatrixPolicy_>;
  using EgoMotionType         = env::EgoMotion<CovarianceMatrixPolicy_>;

  /// \brief Measurement update using a KalmanFilter
  /// \tparam UpdateMode_          Update mode tag (defaults to the policy-appropriate mode)
  /// \tparam ObservationModels_   One or more observation model types (composed if multiple)
  /// \param[in]     filter             The filter instance
  /// \param[in]     egoMotion          Ego motion of the sensor platform, forwarded to each observation model
  /// \param[in,out] motionModel        The motion model whose state and covariance are updated
  /// \param[in]     observationModels  Observation models carrying measurement z and covariance R
  /// \note Defensive skips are silent: degenerate measurement rows are dropped and a stacked R
  ///       that cannot be factorized/decomposed skips the whole update — the void return gives
  ///       the caller no indication of a partial or skipped update.
  template <typename UpdateMode_ = DefaultUpdateMode, typename... ObservationModels_>
  static void run(const KalmanFilterType& filter,
                  const EgoMotionType&    egoMotion,
                  MotionModel_&           motionModel,
                  const ObservationModels_&... observationModels);

  /// \brief Measurement update using an InformationFilter
  /// \tparam UpdateMode_          Update mode tag (defaults to the policy-appropriate mode)
  /// \tparam ObservationModels_   One or more observation model types (composed if multiple)
  /// \param[in]     filter             The filter instance
  /// \param[in]     egoMotion          Ego motion of the sensor platform, forwarded to each observation model
  /// \param[in,out] motionModel        The motion model whose information vector and matrix are updated
  /// \param[in]     observationModels  Observation models carrying measurement z and covariance R
  /// \note Defensive skips are silent: degenerate measurement rows are dropped and a stacked R
  ///       that cannot be inverted/factorized skips the whole update — the void return gives
  ///       the caller no indication of a partial or skipped update.
  template <typename UpdateMode_ = DefaultUpdateMode, typename... ObservationModels_>
  static void run(const InformationFilterType& filter,
                  const EgoMotionType&         egoMotion,
                  MotionModel_&                motionModel,
                  const ObservationModels_&... observationModels);

private:
  /// \brief Recursion end of the observation stacking
  /// \tparam TotalDimZ_  Total stacked measurement dimension
  /// \tparam DimX_       State dimension
  /// \tparam Offset_     Current row offset in the stacked containers
  template <sint32 TotalDimZ_, sint32 DimX_, sint32 Offset_>
  static void stackObservations(math::Vector<value_type, TotalDimZ_>&        innovation,
                                math::Matrix<value_type, TotalDimZ_, DimX_>& H,
                                math::SquareMatrix<value_type, TotalDimZ_>&  R,
                                const math::Vector<value_type, DimX_>&       state,
                                const EgoMotionType&                         egoMotion);

  /// \brief Stack innovation, Jacobian and covariance blocks of the observation models
  /// \tparam TotalDimZ_  Total stacked measurement dimension
  /// \tparam DimX_       State dimension
  /// \tparam Offset_     Current row offset in the stacked containers
  /// \tparam First_      First observation model type
  /// \tparam Rest_       Remaining observation model types
  /// \param[in,out] innovation  Stacked innovation vector to be filled
  /// \param[in,out] H           Stacked measurement Jacobian to be filled
  /// \param[in,out] R           Stacked (block-diagonal) measurement covariance to be filled
  /// \param[in]     state       State vector (state space) the models are evaluated at
  /// \param[in]     egoMotion   Ego motion of the sensor platform, forwarded to each observation model
  /// \param[in]     first       First observation model
  /// \param[in]     rest        Remaining observation models
  template <sint32 TotalDimZ_, sint32 DimX_, sint32 Offset_, typename First_, typename... Rest_>
  static void stackObservations(math::Vector<value_type, TotalDimZ_>&        innovation,
                                math::Matrix<value_type, TotalDimZ_, DimX_>& H,
                                math::SquareMatrix<value_type, TotalDimZ_>&  R,
                                const math::Vector<value_type, DimX_>&       state,
                                const EgoMotionType&                         egoMotion,
                                const First_&                                first,
                                const Rest_&... rest);
};

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // F6A7B8C9_1D2E_4F3A_E4B5_6C7D8E9F0A1B
