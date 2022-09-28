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
template <typename FloatType>
class Point3d: public Vector<FloatType, 3>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using Vector<FloatType, 3>::Vector;

  /// \brief Construct a new Point 3d< Float Type> object
  /// \param[in] other A base class object
  Point3d<FloatType>(const Vector<FloatType, 3>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new Point 3d< Float Type> object
  /// \param[in] x  Value for x
  /// \param[in] y  Value for y
  Point3d<FloatType>(const FloatType x, const FloatType y, const FloatType z);

  /// \brief Read access to x value
  /// \return FloatType 
  auto x() const -> FloatType;

  /// \brief Read access to y value
  /// \return FloatType 
  auto y() const -> FloatType;

  /// \brief Read access to z value
  /// \return FloatType 
  auto z() const -> FloatType;

  /// \brief Write access to x value
  /// \return FloatType 
  auto x() -> FloatType&;

  /// \brief Write access to y value
  /// \return FloatType 
  auto y() -> FloatType&;

  /// \brief Write access to z value
  /// \return FloatType 
  auto z() -> FloatType&;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  /// \brief hide inherited operator[] to prevent wrong access
  using Vector<FloatType, 3>::operator[];
};

template <typename FloatType>
Point3d<FloatType>::Point3d(const Vector<FloatType, 3>& other)
    : Vector<FloatType, 3>{other}
{
}

template <typename FloatType>
Point3d<FloatType>::Point3d(const FloatType x, const FloatType y, const FloatType z)
    : Vector<FloatType, 3>()
{
  this->x() = x;
  this->y() = y;
  this->z() = z;
}

template <typename FloatType>
inline auto Point3d<FloatType>::x() const -> FloatType
{
  return this->operator[](0);
}

template <typename FloatType>
inline auto Point3d<FloatType>::y() const -> FloatType
{
  return this->operator[](1);
}

template <typename FloatType>
inline auto Point3d<FloatType>::z() const -> FloatType
{
  return this->operator[](2);
}

template <typename FloatType>
inline auto Point3d<FloatType>::x() -> FloatType&
{
  return this->operator[](0);
}

template <typename FloatType>
inline auto Point3d<FloatType>::y() -> FloatType&
{
  return this->operator[](1);
}

template <typename FloatType>
inline auto Point3d<FloatType>::z() -> FloatType&
{
  return this->operator[](2);
}

} // namespace math
} // namespace tracking

#endif // D190DD96_01E1_4A50_8090_B1337EB3B42E
