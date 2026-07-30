#ifndef F6A7B8C9_1D2E_4F3A_B4C5_6D7E8F9A0B1C
#define F6A7B8C9_1D2E_4F3A_B4C5_6D7E8F9A0B1C

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/point2d.h"
#include <cmath>

namespace tracking
{
namespace math
{

/// \brief Constant 2D rotation by a fixed angle theta, caching cos/sin
///
/// Shared rotation helper for SE(2) transforms used by observation::SensorMountingPose and
/// env::EgoMotion, so the rotation algebra has a single source of truth.
///
/// \tparam ValueType_ The atomic floating-point type of internal elements
template <typename ValueType_>
class Rotation2D
{
public:
  using value_type = ValueType_;
  using Point2d    = math::Point2d<value_type>;

  // rule of 5 declarations
  Rotation2D()                                         = default;
  Rotation2D(const Rotation2D&)                        = default;
  Rotation2D(Rotation2D&&) noexcept                    = default;
  auto operator=(const Rotation2D&) -> Rotation2D&     = default;
  auto operator=(Rotation2D&&) noexcept -> Rotation2D& = default;
  ~Rotation2D()                                        = default;

  /// \brief Create a rotation from an angle, computing cos/sin
  /// \param[in] yaw Rotation angle theta [rad]
  /// \return Rotation2D representing R(theta)
  static auto FromAngle(const value_type yaw) -> Rotation2D
  {
    Rotation2D rotation{};
    rotation._cos = std::cos(yaw);
    rotation._sin = std::sin(yaw);
    return rotation;
  }

  /// \brief Create a rotation from an already-known cos/sin pair, avoiding recomputation
  /// \param[in] c cos(theta)
  /// \param[in] s sin(theta)
  /// \return Rotation2D representing R(theta)
  static auto FromCosSin(const value_type c, const value_type s) -> Rotation2D
  {
    Rotation2D rotation{};
    rotation._cos = c;
    rotation._sin = s;
    return rotation;
  }

  /// \brief Apply the forward rotation R(theta) * v
  /// \param[in] dx X component of v
  /// \param[in] dy Y component of v
  /// \return Rotated vector R(theta) * v
  [[nodiscard]] auto apply(const value_type dx, const value_type dy) const -> Point2d
  {
    return Point2d::FromValues((_cos * dx) - (_sin * dy), (_sin * dx) + (_cos * dy));
  }

  /// \brief Apply the inverse rotation R(theta)^T * v
  /// \param[in] dx X component of v
  /// \param[in] dy Y component of v
  /// \return Rotated vector R(theta)^T * v
  [[nodiscard]] auto applyTranspose(const value_type dx, const value_type dy) const -> Point2d
  {
    return Point2d::FromValues((_cos * dx) + (_sin * dy), (-_sin * dx) + (_cos * dy));
  }

  /// \brief Read access to the cached cosine
  /// \return cos(theta)
  [[nodiscard]] auto cos() const -> value_type { return _cos; }

  /// \brief Read access to the cached sine
  /// \return sin(theta)
  [[nodiscard]] auto sin() const -> value_type { return _sin; }

private:
  value_type _cos{static_cast<value_type>(1)};
  value_type _sin{static_cast<value_type>(0)};
};

} // namespace math
} // namespace tracking

#endif // F6A7B8C9_1D2E_4F3A_B4C5_6D7E8F9A0B1C
