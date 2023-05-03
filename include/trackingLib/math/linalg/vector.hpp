#ifndef FE49A15E_40AF_485A_A47C_F00025FCB4E2
#define FE49A15E_40AF_485A_A47C_F00025FCB4E2

#include "math/linalg/vector.h"
#include "math/linalg/matrix.hpp"
#include <cmath>

namespace tracking::math
{

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::FromList(
    const std::initializer_list<ValueType_>& list) -> Vector
{
  assert(list.size() == Size_);

  Vector tmp;
  auto iter = tmp.data().begin();
  std::copy(list.begin(), list.end(), iter);
  return tmp;
}

template <typename ValueType_, sint32 Size_>
static inline auto Vector<ValueType_, Size_>::Zeros() -> Vector
{
  return Vector{Matrix::Zeros()};
}
  
template <typename ValueType_, sint32 Size_>
static inline auto Vector<ValueType_, Size_>::Ones() -> Vector
{
  return Vector{Matrix::Ones()};
}

template <typename ValueType_, sint32 Size_>
template <sint32 Row_>
static inline auto Vector<ValueType_, Size_>::UnitVector() -> Vector
{
  static_assert(Row_ >= 0 && Row_ < Size_);

  auto tmp{Vector::Zeros()};
  tmp.at_unsafe(Row_) = static_cast<ValueType_>(1.0);
  return tmp;
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::operator[](sint32 idx) const -> tl::expected<ValueType_, typename Matrix::Errors>
{
  return Matrix::operator()(idx, 0);
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::operator[](sint32 idx) -> tl::expected<std::reference_wrapper<ValueType_>, typename Matrix::Errors>
{
  return Matrix::operator()(idx, 0);
}

template <typename ValueType_, sint32 Size_>
inline auto Vector<ValueType_, Size_>::operator*(const Vector& other) const -> ValueType_
{
  auto sum = static_cast<ValueType_>(0);
  for(auto idx=0; idx<Size_; ++idx)
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
  return std::sqrt(normSq());
}

template <typename ValueType_, sint32 Size_>
template <typename U, std::enable_if_t<std::is_floating_point<U>::value, int>>
inline void Vector<ValueType_, Size_>::normalize()
{
  auto  length = norm();
  this->operator/=(length);
}

template <typename ValueType_, sint32 Size_>
template <typename U, std::enable_if_t<std::is_floating_point<U>::value, int>>
inline auto Vector<ValueType_, Size_>::normalize() const -> Vector
{
  Vector tmp(*this);
  tmp.normalize();
  return tmp;
}

}

#endif // FE49A15E_40AF_485A_A47C_F00025FCB4E2
