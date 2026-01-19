#ifndef C8DBF1C8_E1D4_4EC0_A45D_308AE654A1DC
#define C8DBF1C8_E1D4_4EC0_A45D_308AE654A1DC

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace motion
{

/// \brief Traits structure for motion model type information
///
/// This template struct provides type information and constants for motion models.
/// It serves as a central location for accessing common type definitions and dimensional
/// information that are used throughout the motion model hierarchy.
///
/// The traits structure is used by motion models to:
/// - Define the covariance matrix policy type
/// - Extract the floating-point type from the policy
/// - Access the state definition structure
/// - Provide the state dimension as a compile-time constant
///
/// \tparam CovarianceMatrixPolicy_ Policy type defining the covariance matrix implementation
/// \tparam StateDef_ State definition structure containing state variable enumeration
///
/// Usage example:
/// \code
/// // Define traits for a motion model
/// using MyTraits = MotionModelTraits<FullCovarianceMatrixPolicy<float32>, StateDefCV>;
///
/// // Access type information
/// using FloatType = MyTraits::FloatType; // float32
/// constexpr sint32 stateSize = MyTraits::Size; // StateDefCV::NUM_STATE_VARIABLES
/// \endcode
///
/// \see CovarianceMatrixPolicyBase
/// \see FullCovarianceMatrixPolicy
/// \see FactoredCovarianceMatrixPolicy
/// \see StateDefCV
/// \see StateDefCA
template <typename CovarianceMatrixPolicy_, typename StateDef_>
struct MotionModelTraits
{
  /// \brief Covariance matrix policy type
  /// The policy type that defines how covariance matrices are implemented
  using CovarianceMatrixPolicy = CovarianceMatrixPolicy_;

  /// \brief Floating-point type for calculations
  /// Extracted from the covariance matrix policy
  using FloatType = typename CovarianceMatrixPolicy::FloatType;

  /// \brief State definition structure
  /// Contains enumeration of state variables and their count
  using StateDef = StateDef_;

  /// \brief State dimension
  /// Compile-time constant representing the number of state variables
  static constexpr sint32 Size = StateDef::NUM_STATE_VARIABLES;
};

} // namespace motion
} // namespace tracking

#endif // C8DBF1C8_E1D4_4EC0_A45D_308AE654A1DC
