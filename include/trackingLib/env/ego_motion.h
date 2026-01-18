#ifndef EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8
#define EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/point2d.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace env
{

// TODO(matthias): add interface contract

/// \brief Ego motion compensation for moving sensor platforms
///
/// This class implements ego motion compensation for object tracking systems where the sensor
/// platform (typically a vehicle) is moving. It handles the geometric and kinematic transformations
/// required to compensate for the ego vehicle's motion when tracking objects in the environment.
///
/// The implementation is based on circular motion equations and includes:
/// - Position compensation for objects relative to moving ego vehicle
/// - Direction compensation for velocity vectors
/// - Error propagation for uncertainty in motion parameters using UDU decomposition
/// - Special handling for small angular velocities to avoid numerical issues
/// - Support for both full and factored covariance matrix representations
///
/// Mathematical Background:
/// - Circular motion with yaw rate ω and speed v for time T
/// - Secant approximation for small angular displacements
/// - First-order error propagation for uncertainty in motion parameters
/// - UDU decomposition for numerical stability in covariance calculations
/// - Specialized equations for ω→0 limit to maintain numerical stability
///
/// \tparam CovarianceMatrixType_ Template template for covariance matrix type (CovarianceMatrixFull or CovarianceMatrixFactored)
/// \tparam FloatType_ Floating-point type for computations (float32, float64)
template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType_, typename FloatType_>
class EgoMotion
{
public:
  /// \brief State indices for displacement vector
  ///
  /// Defines the order of states in the displacement vector:
  /// - DS_X: X-position displacement
  /// - DS_Y: Y-position displacement
  /// - DS_PSI: Yaw angle displacement (Δψ)
  enum DisplacementStates
  {
    DS_X = 0,        ///< X-position displacement index
    DS_Y,            ///< Y-position displacement index
    DS_PSI,          ///< Yaw angle displacement index
    DS_NUM_VARIABLES ///< Total number of displacement states
  };

  /// \brief Displacement vector type
  ///
  /// Vector containing [Δx, Δy, Δψ] displacements
  using DisplacementVec = math::Vector<FloatType_, DS_NUM_VARIABLES>;

  /// \brief Displacement covariance matrix type
  ///
  /// Covariance matrix for displacement uncertainties (3×3)
  using DisplacementCov = CovarianceMatrixType_<FloatType_, DS_NUM_VARIABLES>;

  /// \brief Inertial Motion parameters structure
  ///
  /// Contains the inertial motion parameters used in circular motion equations and error propagation.
  /// These parameters include:
  /// - Velocity (v) and its uncertainty (σ_v)
  /// - Acceleration (a) and its uncertainty (σ_a)
  /// - Yaw rate (ω) and its uncertainty (σ_ω)
  ///
  /// These parameters are used to calculate the displacement (dx, dy, dφ) and their covariance
  /// using the equations:
  /// @f[
  ///   dφ = ω·T
  ///   c = \frac{2·v·T + a·T^2}{dφ}·\sin\frac{dφ}{2}
  ///   dx = c·\cos\frac{dφ}{2}
  ///   dy = c·\sin\frac{dφ}{2}
  /// @f]
  ///
  /// For small angular velocities (ω→0), simplified equations (linear motion) are used to avoid numerical issues.
  struct InertialMotion
  {
    FloatType_ v{};      ///< Velocity [m/s]
    FloatType_ a{};      ///< Acceleration [m/s²]
    FloatType_ w{};      ///< Yaw rate [rad/s]
    FloatType_ sv{1e-6}; ///< Velocity uncertainty (standard deviation) [m/s]
    FloatType_ sa{1e-6}; ///< Acceleration uncertainty (standard deviation) [m/s²]
    FloatType_ sw{1e-6}; ///< Yaw rate uncertainty (standard deviation) [rad/s]
  };

  /// \brief Vehicle geometry parameters
  ///
  /// Contains physical dimensions and geometric properties of the ego vehicle
  struct Geometry
  {
    FloatType_ width{};  ///< Vehicle width [m]
    FloatType_ length{}; ///< Vehicle length [m]
    FloatType_ height{}; ///< Vehicle height [m]

    FloatType_ distCog2Ego{};            ///< Distance from center of gravity to ego reference point [m]
    FloatType_ distFrontAxle2Ego{};      ///< Distance from front axle to ego reference point [m]
    FloatType_ distFrontAxle2RearAxle{}; ///< Wheelbase (distance between front and rear axles) [m]
  };

  /// \brief Displacement information structure
  ///
  /// Contains displacement vector, covariance, and precomputed trigonometric values
  /// for efficient compensation calculations
  struct Displacement
  {
    DisplacementVec vec{};                            ///< Displacement vector [Δx, Δy, Δψ]
    DisplacementCov cov{DisplacementCov::Identity()}; ///< Displacement covariance matrix
    FloatType_      sinDeltaPsi{0.0};                 ///< sin(Δψ) - precomputed for efficiency
    FloatType_      cosDeltaPsi{1.0};                 ///< cos(Δψ) - precomputed for efficiency
  };

  // rule of 5 declarations
  EgoMotion()                                        = delete;
  EgoMotion(const EgoMotion&)                        = default;
  EgoMotion(EgoMotion&&) noexcept                    = default;
  auto operator=(const EgoMotion&) -> EgoMotion&     = default;
  auto operator=(EgoMotion&&) noexcept -> EgoMotion& = default;
  virtual ~EgoMotion()                               = default;

  /// \brief Constructor with explicit covariance matrix type
  /// \tparam CovarianceMatrixType Template template for covariance matrix type
  /// \param motion Motion parameters
  /// \param geometry Vehicle geometry
  /// \param dt Time interval
  EgoMotion(const InertialMotion& motion, const Geometry& geometry, const FloatType_ dt)
      : _motion(motion)
      , _geometry(geometry)
      , _dt(dt)
  {
    calcDisplacement();
  }

  /// \brief Get the time interval for this ego motion compensation
  /// \return Time interval Δt [s]
  auto getDeltaTime() const -> FloatType_ { return _dt; }

  /// \brief Get the displacement information for center of gravity
  /// \return Constant reference to displacement structure
  auto getDisplacementCog() const -> const Displacement& { return _displacementCog; }

  /// \brief Get the vehicle geometry parameters
  /// \return Constant reference to geometry structure
  auto getGeometry() const -> const Geometry& { return _geometry; }

  /// \brief Compensate object position for ego vehicle motion
  ///
  /// Transforms object positions from old ego coordinate system to new ego coordinate system
  /// by applying the inverse of the ego vehicle's motion.
  ///
  /// \param[out] posXNewEgo X-position in new ego coordinate system [m]
  /// \param[out] posYNewEgo Y-position in new ego coordinate system [m]
  /// \param[in] posXOldEgo X-position in old ego coordinate system [m]
  /// \param[in] posYOldEgo Y-position in old ego coordinate system [m]
  void compensatePosition(FloatType_&      posXNewEgo,
                          FloatType_&      posYNewEgo,
                          const FloatType_ posXOldEgo,
                          const FloatType_ posYOldEgo) const;

  /// \brief Compensate direction vector for ego vehicle motion
  ///
  /// Transforms velocity/direction vectors from old ego coordinate system to new ego coordinate system
  /// by applying the inverse of the ego vehicle's rotational motion.
  ///
  /// \param[out] dxNewEgo X-component of direction in new ego coordinate system
  /// \param[out] dyNewEgo Y-component of direction in new ego coordinate system
  /// \param[in] dxOldEgo X-component of direction in old ego coordinate system
  /// \param[in] dyOldEgo Y-component of direction in old ego coordinate system
  void compensateDirection(FloatType_&      dxNewEgo,
                           FloatType_&      dyNewEgo,
                           const FloatType_ dxOldEgo,
                           const FloatType_ dyOldEgo) const;

  /// \brief Get the motion parameters
  /// \return Constant reference to motion parameters structure
  auto getMotion() const -> const InertialMotion& { return _motion; }

  // clang-format off
  TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  [[nodiscard]] auto isLinearMotion() const -> bool;

  /// \brief Compute displacement from internal motion parameters
  ///
  /// Calculates the displacement (Δx, Δy, Δψ) and covariance using the circular motion equations
  //  or the linear motion for small angular velocities.
  void calcDisplacement();

  /// \brief Calculate Jacobian for linear motion case
  ///
  /// Computes the Jacobian matrix for linear motion displacement with respect to motion parameters.
  ///
  /// @param[in] motion Input motion parameters
  /// @param[in] dt Time interval for the motion
  /// @return Jacobian matrix (3x3)
  static auto calcLinearMotionJacobian(const InertialMotion& motion, FloatType_ dt) -> math::SquareMatrix<FloatType_, 3, true>;

  /// \brief Calculate Jacobian for circular motion case
  ///
  /// Computes the Jacobian matrix for circular motion displacement with respect to motion parameters.
  ///
  /// @param[in] motion Input motion parameters
  /// @param[in] dt Time interval for the motion
  /// @return Jacobian matrix (3x3)
  static auto calcCircularMotionJacobian(const InertialMotion& motion, FloatType_ dt) -> math::SquareMatrix<FloatType_, 3, true>;

  /// \brief Calculate displacement vector
  ///
  /// Computes the displacement vector [Δx, Δy, Δψ] from motion parameters.
  ///
  /// @param[out] displacement Output displacement structure
  /// @param[in] motion Input motion parameters
  /// @param[in] dt Time interval for the motion
  static void calcDisplacementVector(Displacement& displacement, const InertialMotion& motion, FloatType_ dt);

  /// \brief Calculate displacement covariance
  ///
  /// Computes the displacement covariance matrix using the simplified UDU approach.
  /// This method uses the generic covariance matrix interface to support both full and factored representations.
  ///
  /// @param[out] displacement Output displacement structure with computed covariance
  /// @param[in] J Jacobian matrix
  /// @param[in] motion Input motion parameters
  static void calcDisplacementCovariance(Displacement&                                  displacement,
                                         const math::SquareMatrix<FloatType_, 3, true>& J,
                                         const InertialMotion&                          motion);

  using Point2d = math::Point2d<FloatType_>; ///< 2D point type for geometric calculations
  InertialMotion _motion{};                  ///< Motion parameters (velocity, acceleration, yaw rate and uncertainties)
  Geometry       _geometry{};                ///< Vehicle geometry parameters
  Displacement   _displacementCog{};         ///< Displacement information for center of gravity
  FloatType_     _dt{};                      ///< Time interval for this motion compensation [s]
};

} // namespace env
} // namespace tracking

#endif // EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8
