#ifndef BA34774A_D1AA_44D4_BAD2_1845716F4E58
#define BA34774A_D1AA_44D4_BAD2_1845716F4E58

#include "math/linalg/diagonal_matrix.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
inline DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{}
{
  // copy diagonal elements from other
  for (sint32 idx = 0; idx < Size; ++idx)
  {
    SquareMatrix<FloatType, Size>::operator()(idx, idx) = other(idx, idx);
  }
}

template <typename FloatType, sint32 Size>
inline DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const std::initializer_list<FloatType>& list)
    : SquareMatrix<FloatType, Size>{}
{
  assert((list.size() == Size) && "Mismatching size of intializer list");

  // fill diagonal elements
  sint32 idx = 0;
  for (auto val : list)
  {
    this->operator[](idx++) = val;
  }
}

template <typename FloatType, sint32 Size>
inline DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list)
    : SquareMatrix<FloatType, Size>{list}
{
  // zero anti-diagonal elements
  for (sint32 row = 0; row < Size; ++row)
  {
    for (sint32 col = row + 1; col < Size; ++col)
    {
      this->operator()(row, col) = static_cast<FloatType>(0.0);
      this->operator()(col, row) = static_cast<FloatType>(0.0);
    }
  }
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
    this->operator[](dstIdx) = block[srcIdx];
    ++dstIdx;
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
inline auto DiagonalMatrix<FloatType, Size>::operator*(const Matrix<FloatType, Size, Cols>& mat) -> Matrix<FloatType, Size, Cols>
{
  // each row is multiplied by the corresponding diagonal row element
  auto result{mat};
  for (auto row = 0; row < Size; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) *= this->operator[](col);
    }
  }
  return result;
}

template <typename FloatType, sint32 Size>
template <bool isLower>
inline auto DiagonalMatrix<FloatType, Size>::operator*(const TriangularMatrix<FloatType, Size, isLower>& mat)
    -> TriangularMatrix<FloatType, Size, isLower>
{
  // each row is multiplied by the corresponding diagonal row element
  auto result{mat};
  if (isLower)
  {
    for (auto row = 0; row < Size; ++row)
    {
      for (auto col = 0; col <= row; ++col)
      {
        result(row, col) *= this->operator[](col);
      }
    }
  }
  else
  {
    for (auto row = 0; row < Size; ++row)
    {
      for (auto col = row; col < Size; ++col)
      {
        result(row, col) *= this->operator[](col);
      }
    }
  }
  return result;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator*(const DiagonalMatrix& mat) -> DiagonalMatrix
{
  // element-wise multiplication of the elements on both diagonals
  auto result{*this};
  for (auto idx = 0; idx < Size; ++idx)
  {
    result[idx] *= mat[idx];
  }
  return result;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator*(const FloatType scalar) -> DiagonalMatrix
{
  // element-wise multiplication of the elements on both diagonals
  auto result{*this};
  for (auto idx = 0; idx < Size; ++idx)
  {
    result[idx] *= scalar;
  }
  return result;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator[](const sint32 idx) -> FloatType&
{
  assert(((0 <= idx) && (idx < Size)) && "Index out of bounds");
  return this->operator()(idx, idx);
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator[](const sint32 idx) const -> FloatType
{
  assert(((0 <= idx) && (idx < Size)) && "Index out of bounds");
  return this->operator()(idx, idx);
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
    this->operator[](idx) = static_cast<FloatType>(1.0) / this->operator[](idx);
  }
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::isPositiveDefinite() const -> bool
{
  auto isValid = true;
  for (auto idx = 0; idx < Size; ++idx)
  {
    isValid = isValid && (static_cast<FloatType>(0.0) < this->operator[](idx));
  }
  return isValid;
}

} // namespace math
} // namespace tracking

#endif // BA34774A_D1AA_44D4_BAD2_1845716F4E58
