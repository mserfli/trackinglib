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

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::Matrix(const std::initializer_list<std::initializer_list<ValueType_>>& list)
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

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::setZeros()
{
  data().fill(static_cast<ValueType_>(0));
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::Zeros() -> Matrix
{
  Matrix tmp;
  tmp.setZeros();
  return tmp;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::setOnes()
{
  data().fill(static_cast<ValueType_>(1));
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::Ones() -> Matrix
{
  Matrix tmp;
  tmp.setOnes();
  return tmp;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::print() const
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

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::at_unsafe(sint32 row, sint32 col) const -> ValueType_
{
  if (!IsRowMajor)
  {
    std::swap(row, col);
  }
  return data()[(row * ColsInMem) + col];
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::at_unsafe(sint32 row, sint32 col) -> ValueType_&
{
  if (!IsRowMajor)
  {
    std::swap(row, col);
  }
  return data()[(row * ColsInMem) + col];
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator()(sint32 row, sint32 col) const
    -> tl::expected<ValueType_, Errors>
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

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator()(sint32 row, sint32 col)
    -> tl::expected<std::reference_wrapper<ValueType_>, Errors>
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

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator==(const Matrix& other) const -> bool
{
  return (data() == other.data());
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator==(
    const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other) const -> bool
{
  bool isEqual = true;
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      isEqual = isEqual && (at_unsafe(row, col) == other.at_unsafe(row, col));
    }
  }
  return isEqual;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator+=(const Matrix& other)
{
  const auto& otherData = other.data();
  for (sint32 idx = 0; idx < data().size(); ++idx)
  {
    data()[idx] += otherData[idx];
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator+=(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other)
{
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      auto& val = at_unsafe(row, col);
      val += other.at_unsafe(row, col);
    }
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator-=(const Matrix& other)
{
  const auto& otherData = other.data();
  for (sint32 idx = 0; idx < data().size(); ++idx)
  {
    data()[idx] -= otherData[idx];
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator-=(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other)
{
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      at_unsafe(row, col) -= other.at_unsafe(row, col);
    }
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*=(ValueType_ scalar)
{
  for (auto& val : data())
  {
    val *= scalar;
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename IntType, typename std::enable_if_t<std::is_integral<IntType>::value, bool>>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator/=(IntType scalar)
{
  assert((std::abs(scalar) > static_cast<IntType>(0)) && "division by zero");
  inplace_div_by_int_unsafe(scalar);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename FloatType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool>>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator/=(FloatType scalar)
{
  assert((std::abs(scalar) > std::numeric_limits<FloatType>::min()) && "division by zero");
  inplace_mul_by_inverse_factor_unsafe(scalar);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <bool MajorOrder_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator+(
    const Matrix<ValueType_, Rows_, Cols_, MajorOrder_>& other) const -> Matrix
{
  Matrix res{*this};
  res += other;
  return res;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <bool MajorOrder_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator-(
    const Matrix<ValueType_, Rows_, Cols_, MajorOrder_>& other) const -> Matrix
{
  Matrix res{*this};
  res -= other;
  return res;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*(ValueType_ scalar) const -> Matrix
{
  Matrix res{*this};
  res *= scalar;
  return res;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename IntType, typename std::enable_if_t<std::is_integral<IntType>::value, bool>>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator/(IntType scalar) const -> tl::expected<Matrix, Errors>
{
  if (std::abs(scalar) > static_cast<IntType>(0))
  {
    Matrix res{*this};
    res.inplace_div_by_int_unsafe(scalar);
    return res;
  }
  return tl::unexpected<Errors>{Errors::DIV_BY_ZERO};
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename FloatType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool>>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator/(FloatType scalar) const -> tl::expected<Matrix, Errors>
{
  if (std::abs(scalar) > std::numeric_limits<FloatType>::min())
  {
    Matrix res{*this};
    res.inplace_mul_by_inverse_factor_unsafe(scalar);
    return res;
  }
  return tl::unexpected<Errors>{Errors::DIV_BY_ZERO};
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <sint32 Cols2_, bool IsRowMajor2_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*(
    const Matrix<ValueType_, Cols_, Cols2_, IsRowMajor2_>& other) const -> Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>
{
  using ResultMatrix = Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>;
  ResultMatrix result{ResultMatrix::Zeros()};

  for (auto i = 0; i < ResultMatrix::Rows; ++i)
  {
    for (auto k = 0; k < Cols; ++k)
    {
      for (auto j = 0; j < ResultMatrix::Cols; ++j)
      {
        result.at_unsafe(i, j) += at_unsafe(i, k) * other.at_unsafe(k, j);
      }
    }
  }

  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::transpose() const -> const transpose_type&
{
  return reinterpret_cast<const transpose_type&>(*this);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::transpose() -> transpose_type&
{
  return reinterpret_cast<transpose_type&>(*this);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <sint32 SrcRowSize_,
          sint32 SrcColSize_,
          sint32 SrcRowCount_,
          sint32 SrcColCount_,
          sint32 SrcRowBeg_,
          sint32 SrcColBeg_,
          bool   SrcIsRowMajor_,
          sint32 DstRowBeg_,
          sint32 DstColBeg_>
void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::setBlock(
    const Matrix<ValueType_, SrcRowSize_, SrcColSize_, SrcIsRowMajor_>& block)
{
  static_assert((SrcRowCount_ > 1) && (SrcColCount_ > 1), "use scalar access operator for block copy size == 1");
  static_assert(SrcRowBeg_ + SrcRowCount_ <= SrcRowSize_, "copy to many rows from src");
  static_assert(SrcColBeg_ + SrcColCount_ <= SrcColSize_, "copy to many cols from src");

  static_assert(DstRowBeg_ + SrcRowCount_ <= Rows_, "copy to many rows to dst");
  static_assert(DstColBeg_ + SrcColCount_ <= Cols_, "copy to many cols to dst");

  for (sint32 row = 0; row < SrcRowCount_; ++row)
  {
    for (sint32 col = 0; col < SrcColCount_; ++col)
    {
      this->at_unsafe(DstRowBeg_ + row, DstColBeg_ + col) = block.at_unsafe(SrcRowBeg_ + row, SrcColBeg_ + col);
    }
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename IntType, typename std::enable_if_t<std::is_integral<IntType>::value, bool>>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::inplace_div_by_int_unsafe(IntType scalar)
{
  for (auto& val : data())
  {
    val /= scalar;
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename FloatType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool>>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::inplace_mul_by_inverse_factor_unsafe(FloatType scalar)
{
  const FloatType factor = 1 / scalar;
  this->          operator*=(factor);
}

} // namespace tracking::math
#endif // E4BF6CCD_1BFD_497E_A5CD_75BCD8DD1ED3
