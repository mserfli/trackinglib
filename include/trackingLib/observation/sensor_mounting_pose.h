#ifndef A3B4C5D6_1E2F_4A3B_8C4D_5E6F7A8B9C0D
#define A3B4C5D6_1E2F_4A3B_8C4D_5E6F7A8B9C0D

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/matrix.h"
#include "math/linalg/point2d.h"
#include "math/linalg/rotation2d.h"
#include "math/linalg/vector.h" // IWYU pragma: keep

namespace tracking
{
namespace observation
{

/// \brief Static SE(2) sensor mounting pose (planar translation + yaw) relative to the tracking frame
///
/// Each sensor is mounted at a fixed, exactly known offset and heading relative to the tracking
/// frame. This class holds that extrinsic and provides the transforms needed to map tracking-frame
/// quantities into the sensor frame, plus a helper to chain the constant rotation into a measurement
/// Jacobian. Default-constructed instances are the identity pose (sensor frame == tracking frame),
/// reproducing the library's previous origin-centered behavior.
///
/// \tparam ValueType_ The atomic floating-point type of internal elements
template <typename ValueType_>
class SensorMountingPose
{
public:
  using value_type = ValueType_;
  using Point2d    = math::Point2d<value_type>;
  using Rotation2D = math::Rotation2D<value_type>;

  // rule of 5 declarations
  SensorMountingPose()                                                 = default;
  SensorMountingPose(const SensorMountingPose&)                        = default;
  SensorMountingPose(SensorMountingPose&&) noexcept                    = default;
  auto operator=(const SensorMountingPose&) -> SensorMountingPose&     = default;
  auto operator=(SensorMountingPose&&) noexcept -> SensorMountingPose& = default;
  ~SensorMountingPose()                                                = default;

  /// \brief Create a sensor mounting pose from a translation and a yaw angle
  /// \param[in] tx  Translation along the tracking-frame X axis [m]
  /// \param[in] ty  Translation along the tracking-frame Y axis [m]
  /// \param[in] yaw Mounting yaw angle relative to the tracking frame [rad]
  /// \return SensorMountingPose with the given translation and yaw
  static auto FromValues(const value_type tx, const value_type ty, const value_type yaw) -> SensorMountingPose
  {
    SensorMountingPose pose{};
    pose._tx       = tx;
    pose._ty       = ty;
    pose._rotation = Rotation2D::FromAngle(yaw);
    return pose;
  }

  /// \brief Transform a tracking-frame position into the sensor frame
  ///
  /// Computes `p_s = R(theta)^T * (p_t - t)`: translate by the mounting offset, then rotate by the
  /// inverse mounting yaw.
  ///
  /// \param[in] px Tracking-frame X position [m]
  /// \param[in] py Tracking-frame Y position [m]
  /// \return Position expressed in the sensor frame
  [[nodiscard]] auto positionToSensorFrame(const value_type px, const value_type py) const -> Point2d
  {
    return directionToSensorFrame(px - _tx, py - _ty);
  }

  /// \brief Rotate a tracking-frame direction (e.g. velocity) into the sensor frame
  ///
  /// Computes `v_s = R(theta)^T * v_t`. A static mount has no lever-arm term, so directions rotate
  /// but do not translate.
  ///
  /// \param[in] dx Tracking-frame X direction component
  /// \param[in] dy Tracking-frame Y direction component
  /// \return Direction expressed in the sensor frame
  [[nodiscard]] auto directionToSensorFrame(const value_type dx, const value_type dy) const -> Point2d
  {
    return _rotation.applyTranspose(dx, dy);
  }

  /// \brief Chain the constant sensor-frame rotation into a Jacobian column pair
  ///
  /// Post-multiplies the (colX, colY) column pair of every row by `R(theta)^T`. Given local partials
  /// `[Lx, Ly]` w.r.t. the sensor-frame coordinates already stored in those columns, this turns them
  /// into partials w.r.t. the underlying tracking-frame state columns:
  /// `H(r,colX) = Lx*cos(theta) - Ly*sin(theta)`, `H(r,colY) = Lx*sin(theta) + Ly*cos(theta)`.
  ///
  /// \tparam Rows_ Number of Jacobian rows
  /// \tparam Cols_ Number of Jacobian columns
  /// \param[in,out] jacobian Jacobian matrix updated in place
  /// \param[in] colX Column index holding the local partial w.r.t. the sensor-frame X coordinate
  /// \param[in] colY Column index holding the local partial w.r.t. the sensor-frame Y coordinate
  template <sint32 Rows_, sint32 Cols_>
  void rotateJacobianColumns(math::Matrix<value_type, Rows_, Cols_>& jacobian, const sint32 colX, const sint32 colY) const
  {
    for (sint32 row = 0; row < Rows_; ++row)
    {
      const value_type lx      = jacobian.at_unsafe(row, colX);
      const value_type ly      = jacobian.at_unsafe(row, colY);
      const auto       rotated = _rotation.apply(lx, ly);

      jacobian.at_unsafe(row, colX) = rotated.x();
      jacobian.at_unsafe(row, colY) = rotated.y();
    }
  }

  /// \brief Read access to the mounting yaw cosine
  /// \return cos(theta)
  [[nodiscard]] auto cosYaw() const -> value_type { return _rotation.cos(); }

  /// \brief Read access to the mounting yaw sine
  /// \return sin(theta)
  [[nodiscard]] auto sinYaw() const -> value_type { return _rotation.sin(); }

  /// \brief Read access to the mounting translation X component
  /// \return Translation along the tracking-frame X axis [m]
  [[nodiscard]] auto tx() const -> value_type { return _tx; }

  /// \brief Read access to the mounting translation Y component
  /// \return Translation along the tracking-frame Y axis [m]
  [[nodiscard]] auto ty() const -> value_type { return _ty; }

private:
  value_type _tx{static_cast<value_type>(0)};
  value_type _ty{static_cast<value_type>(0)};
  Rotation2D _rotation{};
};

} // namespace observation
} // namespace tracking

#endif // A3B4C5D6_1E2F_4A3B_8C4D_5E6F7A8B9C0D
