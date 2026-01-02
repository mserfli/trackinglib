#ifndef FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
#define FFCF1757_A52C_4BEF_BFD6_2475D08B37C6

/// \file point2d.h
/// \brief 2D point class with named coordinate access
///
/// This file defines Point2d, a 2D point class that inherits from Vector<ValueType_, 2>
/// and provides convenient named access to x and y coordinates.
///
/// \note Point2d is a thin wrapper around Vector that adds semantic meaning
///       and named accessors for 2D geometric operations.

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/vector.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

// TODO(matthias): add downcast from Point3d
// TODO(matthias): add interface contract

/// \brief 2D point with named coordinate access
///
/// Point2d represents a point in 2D space with x and y coordinates.
/// It inherits from Vector<ValueType_, 2> and provides convenient named
/// accessors for the coordinates, making geometric code more readable.
///
/// All Vector operations are available, plus coordinate-specific accessors.
///
/// \tparam ValueType_ The numeric type for coordinates (e.g., float, double)
///
/// Example usage:
/// \code{.cpp}
/// Point2d<double> p = Point2d<double>::FromValues(1.0, 2.0);
/// double x = p.x(); // 1.0
/// double y = p.y(); // 2.0
/// p.x() = 3.0;     // modify x coordinate
/// \endcode
///
/// \see Point3d for 3D points
/// \see Vector for underlying vector operations
template <typename ValueType_>
class Point2d: public Vector<ValueType_, 2>
{
public:
  /// \brief Type of the parent Vector class
  using BaseVector = Vector<ValueType_, 2>;

  // unhide ctor of base class to allow implicit call in derived default ctors
  using BaseVector::BaseVector;

  /// \brief Construct a new Point 2d<ValueType_> object
  /// \param[in] other A base class object
  explicit Point2d(const BaseVector& other)
      : BaseVector{other}
  {
  }

  /// \brief Move construct a new Point 2d<ValueType_> object
  /// \param[in] other A base class object
  explicit Point2d(BaseVector&& other) noexcept
      : BaseVector{std::move(other)}
  {
  }

  /// \brief Create a 2D point from coordinate values
  ///
  /// Static factory method to create a Point2d from individual x and y values.
  /// This is the preferred way to construct points with explicit coordinates.
  ///
  /// \param[in] x The x-coordinate value
  /// \param[in] y The y-coordinate value
  /// \return Point2d with the specified coordinates
  ///
  /// Example:
  /// \code{.cpp}
  /// auto point = Point2d<double>::FromValues(1.5, -2.3);
  /// \endcode
  static auto FromValues(const ValueType_ x, const ValueType_ y) -> Point2d;

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

private:
  /// \brief hide inherited at_unsafe to prevent wrong access
  using BaseVector::at_unsafe;
};
;

template <typename ValueType_>
inline auto Point2d<ValueType_>::FromValues(const ValueType_ x, const ValueType_ y) -> Point2d
{
  Point2d tmp{};
  tmp.x() = x;
  tmp.y() = y;
  return tmp;
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::x() const -> ValueType_
{
  return BaseVector::at_unsafe(0);
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::y() const -> ValueType_
{
  return BaseVector::at_unsafe(1);
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::x() -> ValueType_&
{
  return BaseVector::at_unsafe(0);
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::y() -> ValueType_&
{
  return BaseVector::at_unsafe(1);
}

} // namespace math
} // namespace tracking

#endif // FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
