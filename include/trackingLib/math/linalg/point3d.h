#ifndef D190DD96_01E1_4A50_8090_B1337EB3B42E
#define D190DD96_01E1_4A50_8090_B1337EB3B42E

/// \file point3d.h
/// \brief 3D point class with named coordinate access
///
/// This file defines Point3d, a 3D point class that inherits from Vector<ValueType_, 3>
/// and provides convenient named access to x, y, and z coordinates.
///
/// \note Point3d is a thin wrapper around Vector that adds semantic meaning
///       and named accessors for 3D geometric operations.

#include "base/first_include.h"   // IWYU pragma: keep
#include "math/linalg/vector.hpp" // IWYU pragma: keep

namespace tracking
{
namespace math
{

// TODO(matthias): add upcast from Point2d
// TODO(matthias): add interface contract

/// \brief 3D point with named coordinate access
///
/// Point3d represents a point in 3D space with x, y, and z coordinates.
/// It inherits from Vector<ValueType_, 3> and provides convenient named
/// accessors for the coordinates, making geometric code more readable.
///
/// All Vector operations are available, plus coordinate-specific accessors.
///
/// \tparam ValueType_ The numeric type for coordinates (e.g., float, double)
///
/// Example usage:
/// \code{.cpp}
/// Point3d<double> p = Point3d<double>::FromValues(1.0, 2.0, 3.0);
/// double x = p.x(); // 1.0
/// double y = p.y(); // 2.0
/// double z = p.z(); // 3.0
/// p.z() = 4.0;     // modify z coordinate
/// \endcode
///
/// \see Point2d for 2D points
/// \see Vector for underlying vector operations
template <typename ValueType_>
class Point3d: public Vector<ValueType_, 3>
{
public:
  /// \brief Type of the parent Vector class
  using BaseVector = Vector<ValueType_, 3>;

  // unhide ctor of base class to allow implicit call in derived default ctors
  using BaseVector::BaseVector;

  /// \brief Construct a new Point 3d<ValueType_> object
  /// \param[in] other A base class object
  explicit Point3d(const BaseVector& other)
      : BaseVector{other}
  {
  }

  /// \brief Move construct a new Point 3d<ValueType_> object
  /// \param[in] other A base class object
  explicit Point3d(BaseVector&& other) noexcept
      : BaseVector{std::move(other)}
  {
  }

  /// \brief Create a 3D point from coordinate values
  ///
  /// Static factory method to create a Point3d from individual x, y, and z values.
  /// This is the preferred way to construct points with explicit coordinates.
  ///
  /// \param[in] x The x-coordinate value
  /// \param[in] y The y-coordinate value
  /// \param[in] z The z-coordinate value
  /// \return Point3d with the specified coordinates
  ///
  /// Example:
  /// \code{.cpp}
  /// auto point = Point3d<double>::FromValues(1.5, -2.3, 4.7);
  /// \endcode
  static auto FromValues(const ValueType_ x, const ValueType_ y, const ValueType_ z) -> Point3d;

  /// \brief Read access to x-coordinate
  ///
  /// Provides read-only access to the x-coordinate (first component).
  ///
  /// \return The x-coordinate value
  [[nodiscard]] auto x() const -> ValueType_;

  /// \brief Read access to y-coordinate
  ///
  /// Provides read-only access to the y-coordinate (second component).
  ///
  /// \return The y-coordinate value
  [[nodiscard]] auto y() const -> ValueType_;

  /// \brief Read access to z-coordinate
  ///
  /// Provides read-only access to the z-coordinate (third component).
  ///
  /// \return The z-coordinate value
  [[nodiscard]] auto z() const -> ValueType_;

  /// \brief Write access to x-coordinate
  ///
  /// Provides read-write access to the x-coordinate (first component).
  ///
  /// \return Reference to the x-coordinate for modification
  [[nodiscard]] auto x() -> ValueType_&;

  /// \brief Write access to y-coordinate
  ///
  /// Provides read-write access to the y-coordinate (second component).
  ///
  /// \return Reference to the y-coordinate for modification
  [[nodiscard]] auto y() -> ValueType_&;

  /// \brief Write access to z-coordinate
  ///
  /// Provides read-write access to the z-coordinate (third component).
  ///
  /// \return Reference to the z-coordinate for modification
  [[nodiscard]] auto z() -> ValueType_&;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief hide inherited at_unsafe to prevent wrong access
  using BaseVector::at_unsafe;
};

template <typename ValueType_>
inline auto Point3d<ValueType_>::FromValues(const ValueType_ x, const ValueType_ y, const ValueType_ z) -> Point3d
{
  Point3d tmp{};
  tmp.x() = x;
  tmp.y() = y;
  tmp.z() = z;
  return tmp;
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::x() const -> ValueType_
{
  return this->at_unsafe(0);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::y() const -> ValueType_
{
  return this->at_unsafe(1);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::z() const -> ValueType_
{
  return this->at_unsafe(2);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::x() -> ValueType_&
{
  return this->at_unsafe(0);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::y() -> ValueType_&
{
  return this->at_unsafe(1);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::z() -> ValueType_&
{
  return this->at_unsafe(2);
}

} // namespace math
} // namespace tracking

#endif // D190DD96_01E1_4A50_8090_B1337EB3B42E
