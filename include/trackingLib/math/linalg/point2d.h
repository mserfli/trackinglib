#ifndef FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
#define FFCF1757_A52C_4BEF_BFD6_2475D08B37C6

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/vector.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

// TODO(matthias): add downcast from Point3d
// TODO(matthias): add interface contract
template <typename ValueType_>
class Point2d: public Vector<ValueType_, 2>
{
public:
  using BaseVector = Vector<ValueType_, 2>; ///< type of the parent class

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

  /// \brief Construct a new Point 2d<ValueType_> object
  /// \param[in] x  Value for x
  /// \param[in] y  Value for y
  static auto FromValues(const ValueType_ x, const ValueType_ y) -> Point2d;

  /// \brief Read access to x value
  /// \return ValueType_
  [[nodiscard]] auto x() const -> ValueType_;

  /// \brief Read access to y value
  /// \return ValueType_
  [[nodiscard]] auto y() const -> ValueType_;

  /// \brief Write access to x value
  /// \return ValueType_
  [[nodiscard]] auto x() -> ValueType_&;

  /// \brief Write access to y value
  /// \return ValueType_
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
