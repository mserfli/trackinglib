#ifndef D190DD96_01E1_4A50_8090_B1337EB3B42E
#define D190DD96_01E1_4A50_8090_B1337EB3B42E

#include "base/first_include.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

// TODO(matthias): add upcast from Point2d
// TODO(matthias): add interface contract
template <typename ValueType_>
class Point3d: public Vector<ValueType_, 3>
{
public:
  using Vector = Vector<ValueType_, 3>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using Vector::Vector;

  /// \brief Construct a new Point 3d< Float Type> object
  /// \param[in] x  Value for x
  /// \param[in] y  Value for y
  /// \param[in] z  Value for z
  static auto FromValues(const ValueType_ x, const ValueType_ y, const ValueType_ z) -> Point3d;

  /// \brief Read access to x value
  /// \return ValueType_
  auto x() const -> ValueType_;

  /// \brief Read access to y value
  /// \return ValueType_
  auto y() const -> ValueType_;

  /// \brief Read access to z value
  /// \return ValueType_
  auto z() const -> ValueType_;

  /// \brief Write access to x value
  /// \return ValueType_
  auto x() -> ValueType_&;

  /// \brief Write access to y value
  /// \return ValueType_
  auto y() -> ValueType_&;

  /// \brief Write access to z value
  /// \return ValueType_
  auto z() -> ValueType_&;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief Construct a new Point 3d< Float Type> object
  /// \param[in] other A base class object
  explicit Point3d(const Vector& other) : Vector{other} {}

  /// \brief hide inherited operator[] to prevent wrong access
  using Vector::operator[];
};

template <typename ValueType_>
static inline auto Point3d<ValueType_>::FromValues(const ValueType_ x, const ValueType_ y, const ValueType_ z) -> Point3d
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
  return this->operator[](0);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::y() const -> ValueType_
{
  return this->operator[](1);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::z() const -> ValueType_
{
  return this->operator[](2);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::x() -> ValueType_&
{
  return this->operator[](0);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::y() -> ValueType_&
{
  return this->operator[](1);
}

template <typename ValueType_>
inline auto Point3d<ValueType_>::z() -> ValueType_&
{
  return this->operator[](2);
}

} // namespace math
} // namespace tracking

#endif // D190DD96_01E1_4A50_8090_B1337EB3B42E
