#ifndef E4BF6CCD_1BFD_497E_A5CD_75BCD8DD1ED3
#define E4BF6CCD_1BFD_497E_A5CD_75BCD8DD1ED3

#include "math/linalg/matrix.h"
#include "tl/expected.hpp"
#include <algorithm>
#include <functional>
#include <limits>
#include <type_traits>

namespace tracking::math
{

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
Matrix<ValueType, Rows, Cols, IsRowMajor>::Matrix(const std::initializer_list<std::initializer_list<ValueType>>& list)
{
  assert(list.size() == RowsInMem);
  assert(list.begin()->size() == ColsInMem);

  auto iter = data().begin();
  for (const auto& row : list)
  {
    std::copy(row.begin(), row.end(), iter);
    iter += row.size();
  }
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline void Matrix<ValueType, Rows, Cols, IsRowMajor>::setZeros()
{
  data().fill(static_cast<ValueType>(0));
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
auto Matrix<ValueType, Rows, Cols, IsRowMajor>::Zeros() -> Matrix
{
  Matrix tmp;
  tmp.setZeros();
  return tmp;
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline void Matrix<ValueType, Rows, Cols, IsRowMajor>::setOnes()
{
  data().fill(static_cast<ValueType>(1));
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
auto Matrix<ValueType, Rows, Cols, IsRowMajor>::Ones() -> Matrix
{
  Matrix tmp;
  tmp.setOnes();
  return tmp;
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline void Matrix<ValueType, Rows, Cols, IsRowMajor>::print() const
{
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      std::cout << at_unsafe(row, col) << ", ";
    }
    std::cout << "\n";
  }
  std::cout << "\n" << std::endl;
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::at_unsafe(sint32 row, sint32 col) const -> ValueType
{
  if(!IsRowMajor)
  {
    std::swap(row,col);
  }
  return data()[(row * ColsInMem) + col];
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::at_unsafe(sint32 row, sint32 col) -> ValueType&
{
  if(!IsRowMajor)
  {
    std::swap(row,col);
  }
  return data()[(row * ColsInMem) + col];
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator()(sint32 row, sint32 col) const -> tl::expected<ValueType, Errors>
{
  if (!(row >= 0 && row < Rows))
  {
    return tl::unexpected<Errors>{Errors::INVALID_ACCESS_ROW};
  }
  if (!(col >= 0 && col < Cols))
  {
    return tl::unexpected<Errors>{Errors::INVALID_ACCESS_COL};
  }
  return at_unsafe(row, col);
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator()(sint32 row, sint32 col)
    -> tl::expected<std::reference_wrapper<ValueType>, Errors>
{
  if (!(row >= 0 && row < Rows))
  {
    return tl::unexpected<Errors>{Errors::INVALID_ACCESS_ROW};
  }
  if (!(col >= 0 && col < Cols))
  {
    return tl::unexpected<Errors>{Errors::INVALID_ACCESS_COL};
  }
  return at_unsafe(row, col);
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator+=(const Matrix& other) -> Matrix&
{
  const auto& otherData = other.data();
  for (sint32 idx = 0; idx < data().size(); ++idx)
  {
    data()[idx] += otherData[idx];
  }
  return *this;
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator-=(const Matrix& other) -> Matrix&
{
  const auto& otherData = other.data();
  for (sint32 idx = 0; idx < data().size(); ++idx)
  {
    data()[idx] -= otherData[idx];
  }
  return *this;
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator*=(ValueType scalar) -> Matrix&
{
  for (auto& val : data())
  {
    val *= scalar;
  }
  return *this;
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
template <typename IntType, typename std::enable_if_t<std::is_integral<IntType>::value, bool>>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator/=(IntType scalar)
    -> tl::expected<std::reference_wrapper<Matrix>, Errors>
{
  if (std::abs(scalar) > static_cast<IntType>(0))
  {
    return this->inplace_div_by_int_unsafe(scalar);
  }
  assert(0 && "division by int zero");
  return tl::unexpected<Errors>{Errors::DIV_BY_ZERO};
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
template <typename IntType, typename std::enable_if_t<std::is_integral<IntType>::value, bool>>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::inplace_div_by_int_unsafe(IntType scalar) -> Matrix&
{
  for (auto& val : data())
  {
    val /= scalar;
  }
  return *this;
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
template <typename FloatType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool>>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator/=(FloatType scalar)
    -> tl::expected<std::reference_wrapper<Matrix>, Errors>
{
  if (std::abs(scalar) > std::numeric_limits<FloatType>::min())
  {
    return inplace_mul_by_inverse_factor_unsafe(scalar);
  }
  assert(0 && "division by close to zero value");
  return tl::unexpected<Errors>{Errors::DIV_BY_ZERO};
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
template <typename FloatType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool>>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::inplace_mul_by_inverse_factor_unsafe(FloatType scalar) -> Matrix&
{
  const FloatType factor = 1 / scalar;
  return this->   operator*=(factor);
}

#if 0

//template <typename ValueType>
//template <sint32 Rows2, sint32 Cols2, bool IsRowMajor2>
//inline auto Matrix<ValueType, Rows2, Cols2, IsRowMajor2>::operator==(
//    const Matrix<ValueType, Rows2, Cols2, IsRowMajor2>& other) const -> bool
//{
//  return true;// (data() == other.data());
//}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator==(const Matrix& other) const -> bool
{
  return (data() == other.data());
}

//template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
//inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::operator==(const Matrix& other) const -> bool
//{
//  return (data() == other.data());
//}




#endif

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::transpose() const -> const transpose_type&
{
  return reinterpret_cast<const transpose_type&>(*this);
}

template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
inline auto Matrix<ValueType, Rows, Cols, IsRowMajor>::transpose() -> transpose_type&
{
  return reinterpret_cast<transpose_type&>(*this);
}



} // namespace tracking::math
#endif // E4BF6CCD_1BFD_497E_A5CD_75BCD8DD1ED3
