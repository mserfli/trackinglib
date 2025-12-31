#ifndef D190DD96_01E1_4A50_8090_B1337EB3B42E
#define D190DD96_01E1_4A50_8090_B1337EB3B42E

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/vector.h" // IWYU pragma: keep

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

  /// \brief Construct a new Point 3d<ValueType_> object
  /// \param[in] other A base class object
  explicit Point3d(const Vector& other)
      : Vector{other}
  {
  }

  /// \brief Move construct a new Point 3d<ValueType_> object
  /// \param[in] other A base class object
  explicit Point3d(Vector&& other) noexcept
      : Vector{std::move(other)}
  {
  }

  /// \brief Construct a new Point 3d<ValueType_> object
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

  /// \brief hide inherited at_unsafe to prevent wrong access
  using Vector::at_unsafe;
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
