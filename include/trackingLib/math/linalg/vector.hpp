#ifndef FE49A15E_40AF_485A_A47C_F00025FCB4E2
#define FE49A15E_40AF_485A_A47C_F00025FCB4E2

#include "math/linalg/vector.h"

template <typename FloatType, sint32 Size>
inline tracking::math::Vector<FloatType, Size>::Vector(const Matrix<FloatType, Size, 1>& other)
    : Matrix<FloatType, Size, 1>{other}
{
}

template <typename FloatType, sint32 Size>
template <sint32 Row>
static inline auto tracking::math::Vector<FloatType, Size>::unitVector() -> Vector<FloatType, Size>
{
  auto tmp(Vector<FloatType, Size>::zero());
  tmp[Row] = static_cast<FloatType>(1.0);
  return tmp;
}

template <typename FloatType, sint32 Size>
inline auto tracking::math::Vector<FloatType, Size>::operator[](sint32 idx) const -> FloatType
{
  return this->operator()(idx, 0);
}

template <typename FloatType, sint32 Size>
inline auto tracking::math::Vector<FloatType, Size>::operator[](sint32 idx) -> FloatType&
{
  return this->operator()(idx, 0);
}

template <typename FloatType, sint32 Size>
inline auto tracking::math::Vector<FloatType, Size>::operator*(const Vector<FloatType, Size>& other) const -> FloatType
{
  const Matrix<FloatType, 1, 1> temp{this->transpose() * other};
  return temp(0, 0);
}

template <typename FloatType, sint32 Size>
inline auto tracking::math::Vector<FloatType, Size>::normSq() const -> FloatType
{
  return this->operator*(*this);
}

template <typename FloatType, sint32 Size>
inline auto tracking::math::Vector<FloatType, Size>::norm() const -> FloatType
{
  return std::sqrt(normSq());
}

template <typename FloatType, sint32 Size>
inline void tracking::math::Vector<FloatType, Size>::normalize()
{
  auto  length = norm();
  this->operator/=(length);
}

template <typename FloatType, sint32 Size>
inline auto tracking::math::Vector<FloatType, Size>::normalize() const -> Vector<FloatType, Size>
{
  Vector<FloatType, Size> tmp(*this);
  tmp.normalize();
  return tmp;
}

#endif // FE49A15E_40AF_485A_A47C_F00025FCB4E2
