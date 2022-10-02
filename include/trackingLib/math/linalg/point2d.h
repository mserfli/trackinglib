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
template <typename FloatType>
class Point2d: public Vector<FloatType, 2>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using Vector<FloatType, 2>::Vector;

  /// \brief Construct a new Point 2d< Float Type> object
  /// \param[in] other A base class object
  Point2d(const Vector<FloatType, 2>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new Point 2d< Float Type> object
  /// \param[in] x  Value for x
  /// \param[in] y  Value for y
  Point2d(const FloatType x, const FloatType y);

  /// \brief Read access to x value
  /// \return FloatType
  auto x() const -> FloatType;

  /// \brief Read access to y value
  /// \return FloatType
  auto y() const -> FloatType;

  /// \brief Write access to x value
  /// \return FloatType
  auto x() -> FloatType&;

  /// \brief Write access to y value
  /// \return FloatType
  auto y() -> FloatType&;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief hide inherited operator[] to prevent wrong access
  using Vector<FloatType, 2>::operator[];
};

template <typename FloatType>
Point2d<FloatType>::Point2d(const Vector<FloatType, 2>& other)
    : Vector<FloatType, 2>{other}
{
}

template <typename FloatType>
Point2d<FloatType>::Point2d(const FloatType x, const FloatType y)
    : Vector<FloatType, 2>()
{
  this->x() = x;
  this->y() = y;
}

template <typename FloatType>
inline auto Point2d<FloatType>::x() const -> FloatType
{
  return this->operator[](0);
}

template <typename FloatType>
inline auto Point2d<FloatType>::y() const -> FloatType
{
  return this->operator[](1);
}

template <typename FloatType>
inline auto Point2d<FloatType>::x() -> FloatType&
{
  return this->operator[](0);
}

template <typename FloatType>
inline auto Point2d<FloatType>::y() -> FloatType&
{
  return this->operator[](1);
}

} // namespace math
} // namespace tracking

#endif // FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
