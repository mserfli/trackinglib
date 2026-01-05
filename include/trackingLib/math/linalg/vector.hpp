#ifndef FE49A15E_40AF_485A_A47C_F00025FCB4E2
#define FE49A15E_40AF_485A_A47C_F00025FCB4E2

#include "math/linalg/vector.h"

#include "math/linalg/matrix.hpp" // IWYU pragma: keep
#include <cmath>                  // for sqrt

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::Zeros() -> Vector
{
  return Vector{BaseMatrix::Zeros()};
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::Ones() -> Vector
{
  return Vector{BaseMatrix::Ones()};
}

template <typename ValueType_, sint32 Size_>
template <sint32 Row_>
inline auto Vector<ValueType_, Size_>::UnitVector() -> Vector
{
  static_assert(Row_ >= 0 && Row_ < Size_);

  auto tmp{Vector::Zeros()};
  tmp.at_unsafe(Row_) = static_cast<ValueType_>(1.0);
  return tmp;
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::operator[](sint32 idx) const -> tl::expected<ValueType_, Errors>
{
  return BaseMatrix::operator()(idx, 0);
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::operator[](sint32 idx) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>
{
  return BaseMatrix::operator()(idx, 0);
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::operator*(const Vector& other) const -> ValueType_
{
  auto sum = static_cast<ValueType_>(0);
  for (auto idx = 0; idx < Size_; ++idx)
  {
    sum += at_unsafe(idx) * other.at_unsafe(idx);
  }
  return sum;
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::normSq() const -> ValueType_
{
  return this->operator*(*this);
}

template <typename ValueType_, sint32 Size_>
template <typename U, std::enable_if_t<std::is_floating_point<U>::value, int>>
inline auto Vector<ValueType_, Size_>::norm() const -> ValueType_
{
  if constexpr (Size_ == 1)
  {
    return std::abs(Matrix<ValueType_, 1, 1, true>::at_unsafe(0, 0));
  }
  else
  {
    return std::sqrt(normSq());
  }
}

template <typename ValueType_, sint32 Size_>
template <typename U, std::enable_if_t<std::is_floating_point<U>::value, int>>
inline void Vector<ValueType_, Size_>::normalize()
{
  if constexpr (Size_ == 1)
  {
    // the normalized vector of dim 1 is always 1 or -1, so we just set it accordingly
    at_unsafe(0) = std::copysign(static_cast<ValueType_>(1.0), at_unsafe(0));
  }
  else
  {
    const auto length = norm();
    this->operator/=(length);
  }
}

template <typename ValueType_, sint32 Size_>
template <typename U, std::enable_if_t<std::is_floating_point<U>::value, int>>
inline auto Vector<ValueType_, Size_>::normalize() const -> Vector
{
  Vector tmp(*this);
  tmp.normalize();
  return tmp;
}

} // namespace math
} // namespace tracking

#endif // FE49A15E_40AF_485A_A47C_F00025FCB4E2
