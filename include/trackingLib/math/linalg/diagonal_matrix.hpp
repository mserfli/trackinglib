#ifndef BA34774A_D1AA_44D4_BAD2_1845716F4E58
#define BA34774A_D1AA_44D4_BAD2_1845716F4E58

#include "math/linalg/diagonal_matrix.h"

#include "math/linalg/square_matrix.hpp"
#include "math/linalg/triangular_matrix.hpp"
#include "math/linalg/vector.hpp"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
inline DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const SquareMatrix<FloatType, Size>& other)
{
  // copy diagonal elements from other
  for (sint32 idx = 0; idx < Size; ++idx)
  {
    _data[idx] = other(idx, idx);
  }
}

template <typename FloatType, sint32 Size>
inline DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const std::initializer_list<FloatType>& list)
{
  assert((list.size() == Size) && "Mismatching size of intializer list");

  // fill diagonal elements
  sint32 idx = 0;
  for (auto val : list)
  {
    _data[idx++] = val;
  }
}

template <typename FloatType, sint32 Size>
inline DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list)
{
  assert((list.size() == Size) && "Mismatching size of intializer list");

  // copy diagonal elements from list
  sint32 idx = 0;
  for (const auto& rowList : list)
  {
    assert((rowList.size() == Size) && "Mismatching size of intializer list");
    _data[idx] = *(rowList.begin() + idx);
    ++idx;
  }
}

template <typename FloatType, sint32 Size>
inline void DiagonalMatrix<FloatType, Size>::setIdentity()
{
  for (sint32 idx = 0; idx < Size; ++idx)
  {
    _data[idx] = static_cast<FloatType>(1.0);
  }
}

template <typename FloatType, sint32 Size>
inline static auto DiagonalMatrix<FloatType, Size>::Identity() -> DiagonalMatrix
{
  DiagonalMatrix diag;
  diag.setIdentity();
  return diag;
}

template <typename FloatType, sint32 Size>
template <sint32 SrcSize, sint32 SrcCount, sint32 SrcIdxBeg, sint32 DstIdxBeg>
inline void DiagonalMatrix<FloatType, Size>::setBlock(const DiagonalMatrix<FloatType, SrcSize>& block)
{
  static_assert(SrcCount > 1, "use scalar access operator for block copy size == 1");
  static_assert(SrcIdxBeg + SrcCount <= SrcSize, "copy to many rows from src");

  static_assert(DstIdxBeg + SrcCount <= Size, "copy to many rows to dst");

  sint32 dstIdx = DstIdxBeg;
  for (auto srcIdx = SrcIdxBeg; srcIdx < SrcIdxBeg + SrcCount; ++srcIdx)
  {
    _data[dstIdx++] = block[srcIdx];
  }
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator=(const std::initializer_list<FloatType>& list) -> DiagonalMatrix&
{
  *this = DiagonalMatrix(list);
  return *this;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator=(const std::initializer_list<std::initializer_list<FloatType>>& list)
    -> DiagonalMatrix&
{
  *this = DiagonalMatrix(list);
  return *this;
}

template <typename FloatType, sint32 Size>
template <sint32 Cols>
inline auto DiagonalMatrix<FloatType, Size>::operator*(const Matrix<FloatType, Size, Cols>& mat) const
    -> Matrix<FloatType, Size, Cols>
{
  // each row is multiplied by the corresponding diagonal row element
  Matrix<FloatType, Size, Cols> result{mat};
  for (auto row = 0; row < Size; ++row)
  {
    const FloatType val = _data[row];
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) *= val;
    }
  }
  return result;
}

template <typename FloatType, sint32 Size>
template <bool isLower>
inline auto DiagonalMatrix<FloatType, Size>::operator*(const TriangularMatrix<FloatType, Size, isLower>& mat) const
    -> TriangularMatrix<FloatType, Size, isLower>
{
  // each row is multiplied by the corresponding diagonal row element
  auto result{mat};
  if (isLower)
  {
    for (auto row = 0; row < Size; ++row)
    {
      const FloatType val = _data[row];
      for (auto col = 0; col <= row; ++col)
      {
        result(row, col) *= val;
      }
    }
  }
  else
  {
    for (auto row = 0; row < Size; ++row)
    {
      const FloatType val = _data[row];
      for (auto col = row; col < Size; ++col)
      {
        result(row, col) *= val;
      }
    }
  }
  return result;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator*(const DiagonalMatrix& mat) const -> DiagonalMatrix
{
  // copy *this
  auto result{*this};
  result *= mat;
  return result;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator*(const FloatType scalar) const -> DiagonalMatrix
{
  // copy *this
  auto result{*this};
  result *= scalar;
  return result;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator*=(const DiagonalMatrix& mat) -> DiagonalMatrix&
{
  // element-wise multiplication of the elements on both diagonals
  for (auto idx = 0; idx < Size; ++idx)
  {
    _data[idx] *= mat[idx];
  }
  return *this;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator*=(const FloatType scalar) -> DiagonalMatrix&
{
  // element-wise multiplication of the elements on both diagonals
  for (auto idx = 0; idx < Size; ++idx)
  {
    _data[idx] *= scalar;
  }
  return *this;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::inverse() const -> DiagonalMatrix
{
  DiagonalMatrix tmp{*this};
  tmp.inverse();
  return tmp;
}

template <typename FloatType, sint32 Size>
inline void DiagonalMatrix<FloatType, Size>::inverse()
{
  for (sint32 idx = 0; idx < Size; ++idx)
  {
    assert((static_cast<FloatType>(0.0) < this->operator[](idx)) && "inverse not possible");
    _data[idx] = static_cast<FloatType>(1.0) / _data[idx];
  }
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::isPositiveDefinite() const -> bool
{
  auto isValid = true;
  for (auto idx = 0; idx < Size; ++idx)
  {
    isValid = isValid && (static_cast<FloatType>(0.0) < _data[idx]);
  }
  return isValid;
}

template <typename FloatType, sint32 Size>
inline void DiagonalMatrix<FloatType, Size>::print() const
{
  const SquareMatrix<FloatType, Size> diag{*this};
  diag.print();
}

// ------ non-member functions ---------------------------------------------------------------------------------------------------

template <typename FloatType, sint32 Rows, sint32 Cols>
auto operator*(const Matrix<FloatType, Rows, Cols>& mat, const DiagonalMatrix<FloatType, Cols>& diag)
    -> Matrix<FloatType, Rows, Cols>
{
  // each column is multiplied by the corresponding diagonal column element
  auto result{mat};
  for (auto col = 0; col < Cols; ++col)
  {
    for (auto row = 0; row < Rows; ++row)
    {
      result(row, col) *= diag[col];
    }
  }
  return result;
}

} // namespace math
} // namespace tracking

#endif // BA34774A_D1AA_44D4_BAD2_1845716F4E58
