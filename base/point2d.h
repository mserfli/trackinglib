#ifndef FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
#define FFCF1757_A52C_4BEF_BFD6_2475D08B37C6

#include "base/vector.h"

namespace tracking
{
namespace base
{

template <typename FloatType>
class Point2d: public Vector<FloatType, 2>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using Vector<FloatType, 2>::Vector;

  /// \brief Construct a new Point 2d< Float Type> object
  /// \param[in] other A base class object
  Point2d<FloatType>(const Vector<FloatType, 2>& other) // NOLINT(google-explicit-constructor)
      : Vector<FloatType, 2>{other}
  {
  }

  Point2d<FloatType>(const FloatType x, const FloatType y)
      : Vector<FloatType, 2>()
  {
    this->x() = x;
    this->y() = y;
  }

  auto x() const -> FloatType { return this->operator[](0); }
  auto y() const -> FloatType { return this->operator[](1); }
  auto x() -> FloatType& { return this->operator[](0); }
  auto y() -> FloatType& { return this->operator[](1); }

private:
  using Vector<FloatType, 2>::operator[];
};

} // namespace base
} // namespace tracking

#endif // FFCF1757_A52C_4BEF_BFD6_2475D08B37C6
