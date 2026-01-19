#ifndef DA3325A6_9F6B_4D55_87B2_336CB0BAF21E
#define DA3325A6_9F6B_4D55_87B2_336CB0BAF21E

#include "base/first_include.h" // IWYU pragma: keep
#include "env/ego_motion.h"

namespace tracking
{
namespace motion
{
namespace generic
{

/// \brief Base class for common calculations needed for any prediction
/// \tparam MotionModel_             The underlying motion model
/// \tparam CovarianceMatrixPolicy_  Policy type that defines the covariance matrix implementation
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class PredictCommon
{
public:
  using FloatType     = typename CovarianceMatrixPolicy_::FloatType;
  using EgoMotionType = env::EgoMotion<CovarianceMatrixPolicy_>;

  /// \brief Structure to store the calculations
  struct Storage
  {
    /// \brief Go defines the transformation of the state caused by the ego motion
    typename MotionModel_::StateMatrix Go{};
    /// \brief Ge defines the propagated errors of the ego motion to the state space
    typename MotionModel_::EgoMotionMappingMatrix Ge{};

    /// \brief A defines the state transition from k to k+1 and is calculated as the Jacobian for nonlinear process models
    typename MotionModel_::StateMatrix A{};
    /// \brief Q defines the process noise
    typename MotionModel_::ProcessNoiseDiagMatrix Q{};
    /// \brief G defines the transformation of the process noise to the full state space
    typename MotionModel_::ProcessNoiseMappingMatrix G{};
  };

  /// \brief Runner to calculate the common predict data for the predictor
  /// \param[out] data       Output data storage for all precomputed results
  /// \param[in]  dt         The delta time from last state to predicted state
  /// \param[in]  egoMotion  The known egoMotion from last state to predicted state
  void run(Storage& data, const FloatType dt, const EgoMotionType& egoMotion);
};

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // DA3325A6_9F6B_4D55_87B2_336CB0BAF21E
