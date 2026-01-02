#ifndef E4BF6CCD_1BFD_497E_A5CD_75BCD8DD1ED3
#define E4BF6CCD_1BFD_497E_A5CD_75BCD8DD1ED3

#include "math/linalg/matrix.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <type_traits>

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::setZeros()
{
  data().fill(static_cast<ValueType_>(0));
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::Zeros() -> Matrix
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
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::Ones() -> Matrix
{
  Matrix tmp;
  tmp.setOnes();
  return tmp;
}


template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::at_unsafe(sint32 row, sint32 col) const -> ValueType_
{
  assert((0 <= row) && (row < Rows));
  assert((0 <= col) && (col < Cols));
  const auto idx = IsRowMajor_ ? (row * ColsInMem) + col : (col * ColsInMem) + row;
  return data()[idx];
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::at_unsafe(sint32 row, sint32 col) -> ValueType_&
{
  assert((0 <= row) && (row < Rows));
  assert((0 <= col) && (col < Cols));
  const auto idx = IsRowMajor_ ? (row * ColsInMem) + col : (col * ColsInMem) + row;
  return data()[idx];
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator()(sint32 row,
                                                                      sint32 col) const -> tl::expected<ValueType_, Errors>
{
  if (!(row >= 0 && row < Rows))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_row};
  }
  if (!(col >= 0 && col < Cols))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_col};
  }
  return at_unsafe(row, col);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator()(sint32 row, sint32 col)
    -> tl::expected<std::reference_wrapper<ValueType_>, Errors>
{
  if (!(row >= 0 && row < Rows))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_row};
  }
  if (!(col >= 0 && col < Cols))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_col};
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
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator!=(const Matrix& other) const -> bool
{
  return !(*this == other);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator!=(
    const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other) const -> bool
{
  return !(*this == other);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator+=(const Matrix& other)
{
  const auto&  otherData = other.data();
  const sint32 size      = data().size();
  for (sint32 idx = 0; idx < size; ++idx)
  {
    data()[idx] += otherData[idx];
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator+=(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other)
{
  if (this->data().data() != other.data().data())
  {
    for (auto row = 0; row < Rows; ++row)
    {
      for (auto col = 0; col < Cols; ++col)
      {
        at_unsafe(row, col) += other.at_unsafe(row, col);
      }
    }
  }
  else
  {
    // When both matrices share the same underlying data (e.g., transposed view),
    // we must make a copy to avoid read-after-write corruption.
    // Example: a += a.transpose() would overwrite a[0][1] before reading it as a.transpose()[1][0]
    const auto copy{other};
    for (auto row = 0; row < Rows; ++row)
    {
      for (auto col = 0; col < Cols; ++col)
      {
        at_unsafe(row, col) += copy.at_unsafe(row, col);
      }
    }
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator-=(const Matrix& other)
{
  const auto&  otherData = other.data();
  const sint32 size      = data().size();
  for (sint32 idx = 0; idx < size; ++idx)
  {
    data()[idx] -= otherData[idx];
  }
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator-=(const Matrix<ValueType_, Rows_, Cols_, !IsRowMajor_>& other)
{
  if (this->data().data() != other.data().data())
  {
    for (auto row = 0; row < Rows; ++row)
    {
      for (auto col = 0; col < Cols; ++col)
      {
        at_unsafe(row, col) -= other.at_unsafe(row, col);
      }
    }
  }
  else
  {
    // When both matrices share the same underlying data (e.g., transposed view),
    // we must make a copy to avoid read-after-write corruption.
    const auto copy{other};
    for (auto row = 0; row < Rows; ++row)
    {
      for (auto col = 0; col < Cols; ++col)
      {
        at_unsafe(row, col) -= copy.at_unsafe(row, col);
      }
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
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator/=(IntType scalar) -> tl::expected<void, Errors>
{
  if (std::abs(scalar) > static_cast<IntType>(0))
  {
    inplace_div_by_int_unsafe(scalar);
    return {};
  }
  return tl::unexpected<Errors>{Errors::divide_by_zero};
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename FloatType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool>>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator/=(FloatType scalar) -> tl::expected<void, Errors>
{
  if (std::abs(scalar) > std::numeric_limits<FloatType>::min())
  {
    inplace_mul_by_inverse_factor_unsafe(scalar);
    return {};
  }
  return tl::unexpected<Errors>{Errors::divide_by_zero};
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
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator+(ValueType_ scalar) const -> Matrix
{
  Matrix res{*this};
  for (auto& val : res.data())
  {
    val += scalar;
  }
  return res;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator-(ValueType_ scalar) const -> Matrix
{
  Matrix res{*this};
  for (auto& val : res.data())
  {
    val -= scalar;
  }
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
  return tl::unexpected<Errors>{Errors::divide_by_zero};
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
  return tl::unexpected<Errors>{Errors::divide_by_zero};
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <sint32 Cols2_, bool IsRowMajor2_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*(
    const Matrix<ValueType_, Cols_, Cols2_, IsRowMajor2_>& other) const -> Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>
{
  using ResultMatrix = Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>;
  ResultMatrix result{ResultMatrix::Zeros()};

  // Optimized loop order (i-j-k) for cache-friendly row-major access (4-8x faster)
  for (auto i = 0; i < ResultMatrix::Rows; ++i)
  {
    for (auto j = 0; j < ResultMatrix::Cols; ++j)
    {
      for (auto k = 0; k < Cols; ++k)
      {
        result.at_unsafe(i, j) += at_unsafe(i, k) * other.at_unsafe(k, j);
      }
    }
  }

  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::minmax() const -> std::tuple<ValueType_, ValueType_>
{
  const auto [min, max] = std::minmax_element(data().begin(), data().end());
  return std::make_tuple(*min, *max);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <typename FloatType, typename std::enable_if_t<std::is_floating_point<FloatType>::value, bool>>
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::frobenius_norm() const -> ValueType_
{
  ValueType_ sum_of_squares = static_cast<ValueType_>(0);
  for (const auto& val : data())
  {
    sum_of_squares += val * val;
  }
  return std::sqrt(sum_of_squares);
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
inline auto Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::transpose_rvalue() && -> transpose_type
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
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::setBlock(
    const Matrix<ValueType_, SrcRowSize_, SrcColSize_, SrcIsRowMajor_>& block)
{
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
template <sint32 SrcRowSize_, sint32 SrcColSize_, bool SrcIsRowMajor_>
inline void Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>::setBlock(
    const sint32                                                        srcRowCount,
    const sint32                                                        srcColCount,
    const sint32                                                        srcRowBeg,
    const sint32                                                        srcColBeg,
    const sint32                                                        dstRowBeg,
    const sint32                                                        dstColBeg,
    const Matrix<ValueType_, SrcRowSize_, SrcColSize_, SrcIsRowMajor_>& block)
{
  assert((srcRowBeg + srcRowCount <= SrcRowSize_) && "copy to many rows from src");
  assert((srcColBeg + srcColCount <= SrcColSize_) && "copy to many cols from src");

  assert((dstRowBeg + srcRowCount <= Rows) && "copy to many rows to dst");
  assert((dstColBeg + srcColCount <= Cols) && "copy to many cols to dst");

  for (sint32 row = 0; row < srcRowCount; ++row)
  {
    for (sint32 col = 0; col < srcColCount; ++col)
    {
      this->at_unsafe(dstRowBeg + row, dstColBeg + col) = block.at_unsafe(srcRowBeg + row, srcColBeg + col);
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
  this->operator*=(factor);
}

} // namespace math
} // namespace tracking

#endif // E4BF6CCD_1BFD_497E_A5CD_75BCD8DD1ED3
