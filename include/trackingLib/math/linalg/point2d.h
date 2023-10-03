#ifndef FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
#define FFCF1757_A52C_4BEF_BFD6_2475D08B37C6

#include "base/first_include.h"
#include "math/linalg/vector.h"

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
  using Vector = Vector<ValueType_, 2>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using Vector::Vector;

  /// \brief Construct a new Point 2d< Float Type> object
  /// \param[in] x  Value for x
  /// \param[in] y  Value for y
  static auto FromValues(const ValueType_ x, const ValueType_ y) -> Point2d;

  /// \brief Read access to x value
  /// \return ValueType_
  auto x() const -> ValueType_;

  /// \brief Read access to y value
  /// \return ValueType_
  auto y() const -> ValueType_;

  /// \brief Write access to x value
  /// \return ValueType_
  auto x() -> ValueType_&;

  /// \brief Write access to y value
  /// \return ValueType_
  auto y() -> ValueType_&;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief Construct a new Point 2d< Float Type> object
  /// \param[in] other A base class object
  explicit Point2d(const Vector& other)
      : Vector{other}
  {
  }

  /// \brief hide inherited operator[] to prevent wrong access
  using Vector::operator[];
};

template <typename ValueType_>
static inline auto Point2d<ValueType_>::FromValues(const ValueType_ x, const ValueType_ y) -> Point2d
{
  Point2d tmp{};
  tmp.x() = x;
  tmp.y() = y;
  return tmp;
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::x() const -> ValueType_
{
  return this->operator[](0);
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::y() const -> ValueType_
{
  return this->operator[](1);
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::x() -> ValueType_&
{
  return this->operator[](0);
}

template <typename ValueType_>
inline auto Point2d<ValueType_>::y() -> ValueType_&
{
  return this->operator[](1);
}

} // namespace math
} // namespace tracking

#endif // FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
