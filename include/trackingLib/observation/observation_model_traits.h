#ifndef E5F6A7B8_1C2D_4E3F_9A0B_1C2D3E4F5A6B
#define E5F6A7B8_1C2D_4E3F_9A0B_1C2D3E4F5A6B

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace observation
{

/// \brief Traits structure for observation model type information
///
/// This template struct provides type information and constants for observation models.
/// It mirrors motion::MotionModelTraits and serves as the central location for accessing
/// common type definitions and dimensional information used throughout the observation
/// model hierarchy.
///
/// The traits structure is used by observation models to:
/// - Define the covariance matrix policy type
/// - Extract the floating-point type from the policy
/// - Access the state definition structure the model observes
/// - Provide the measurement dimension as a compile-time constant
///
/// \tparam CovarianceMatrixPolicy_ Policy type defining the covariance matrix implementation
/// \tparam StateDef_ State definition structure containing state variable enumeration
/// \tparam DimZ_ Measurement dimension (compile-time constant, > 0)
///
/// \see motion::MotionModelTraits
/// \see ExtendedObservationModel
template <typename CovarianceMatrixPolicy_, typename StateDef_, sint32 DimZ_>
struct ObservationModelTraits
{
  /// \brief Covariance matrix policy type
  /// The policy type that defines how covariance matrixes are implemented
  using CovarianceMatrixPolicy = CovarianceMatrixPolicy_;

  /// \brief Floating-point type for calculations
  /// Extracted from the covariance matrix policy
  using value_type = typename CovarianceMatrixPolicy::value_type;

  /// \brief State definition structure
  /// Contains enumeration of state variables and their count
  using StateDef = StateDef_;

  /// \brief Measurement dimension
  /// Compile-time constant representing the number of measurement components
  static constexpr sint32 DimZ = DimZ_;
};

} // namespace observation
} // namespace tracking

#endif // E5F6A7B8_1C2D_4E3F_9A0B_1C2D3E4F5A6B
