#ifndef BA34774A_D1AA_44D4_BAD2_1845716F4E58
#define BA34774A_D1AA_44D4_BAD2_1845716F4E58

#include "math/linalg/diagonal_matrix.h"

#include "math/linalg/matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"     // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"            // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::Identity() -> DiagonalMatrix
{
  DiagonalMatrix diag{};
  diag.setIdentity();
  return diag;
}

template <typename ValueType_, sint32 Size_>
template <bool IsRowMajor_>
inline auto DiagonalMatrix<ValueType_, Size_>::FromMatrix(const SquareMatrix<ValueType_, Size_, IsRowMajor_>& other)
    -> DiagonalMatrix
{
  DiagonalMatrix diag{};
  // copy diagonal elements from other
  for (sint32 idx = 0; idx < Size_; ++idx)
  {
    diag.at_unsafe(idx) = other.at_unsafe(idx, idx);
  }
  return diag;
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::FromList(const std::initializer_list<ValueType_>& list) -> DiagonalMatrix
{
  assert((list.size() == Size_) && "Mismatching size of intializer list");

  DiagonalMatrix diag{};
  // fill diagonal elements
  sint32 idx = 0;
  for (auto val : list)
  {
    diag.at_unsafe(idx++) = val;
  }
  return diag;
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::FromList(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> DiagonalMatrix
{
  assert(list.size() == Size_);
  assert(list.begin()->size() == Size_);

  DiagonalMatrix diag{};
  // copy diagonal elements from list
  sint32 idx = 0;
  for (const auto& rowList : list)
  {
    assert((rowList.size() == Size_) && "Mismatching size of intializer list");
    diag.at_unsafe(idx) = *(rowList.begin() + idx);
    ++idx;
  }
  return diag;
}

template <typename ValueType_, sint32 Size_>
inline void DiagonalMatrix<ValueType_, Size_>::setIdentity()
{
  _data.setOnes();
}

template <typename ValueType_, sint32 Size_>
template <sint32 SrcSize_, sint32 SrcCount_, sint32 SrcIdxBeg_, sint32 DstIdxBeg_>
inline void DiagonalMatrix<ValueType_, Size_>::setBlock(const DiagonalMatrix<ValueType_, SrcSize_>& block)
{
  static_assert(SrcCount_ > 1, "use scalar access operator for block copy size == 1");
  static_assert(SrcIdxBeg_ + SrcCount_ <= SrcSize_, "copy to many rows from src");

  static_assert(DstIdxBeg_ + SrcCount_ <= Size_, "copy to many rows to dst");

  sint32 dstIdx = DstIdxBeg_;
  for (auto srcIdx = SrcIdxBeg_; srcIdx < SrcIdxBeg_ + SrcCount_; ++srcIdx)
  {
    _data.at_unsafe(dstIdx++) = block.at_unsafe(srcIdx);
  }
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::operator=(const std::initializer_list<ValueType_>& list) -> DiagonalMatrix&
{
  *this = FromList(list);
  return *this;
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::operator=(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> DiagonalMatrix&
{
  *this = FromList(list);
  return *this;
}

template <typename ValueType_, sint32 Size_>
template <sint32 Cols_, bool IsRowMajor_>
inline auto DiagonalMatrix<ValueType_, Size_>::operator*(const Matrix<ValueType_, Size_, Cols_, IsRowMajor_>& mat) const
    -> Matrix<ValueType_, Size_, Cols_, IsRowMajor_>
{
  // each row is multiplied by the corresponding diagonal row element
  auto result{mat};
  for (auto row = 0; row < Size_; ++row)
  {
    const ValueType_ val = _data.at_unsafe(row);
    for (auto col = 0; col < Cols_; ++col)
    {
      result.at_unsafe(row, col) *= val;
    }
  }
  return result;
}

template <typename ValueType_, sint32 Size_>
template <bool IsLower_, bool IsRowMajor_>
inline auto DiagonalMatrix<ValueType_, Size_>::operator*(const TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>& mat)
    const -> TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>
{
  // each row is multiplied by the corresponding diagonal row element
  auto result{mat};
  if (IsLower_)
  {
    for (auto row = 0; row < Size_; ++row)
    {
      const ValueType_ val = _data.at_unsafe(row);
      for (auto col = 0; col <= row; ++col)
      {
        result.at_unsafe(row, col) *= val;
      }
    }
  }
  else
  {
    for (auto row = 0; row < Size_; ++row)
    {
      const ValueType_ val = _data.at_unsafe(row);
      for (auto col = row; col < Size_; ++col)
      {
        result.at_unsafe(row, col) *= val;
      }
    }
  }
  return result;
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::operator*(const DiagonalMatrix& mat) const -> DiagonalMatrix
{
  // copy *this
  auto result{*this};
  result *= mat;
  return result;
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::operator*(const ValueType_ scalar) const -> DiagonalMatrix
{
  // copy *this
  auto result{*this};
  result *= scalar;
  return result;
}

template <typename ValueType_, sint32 Size_>
inline void DiagonalMatrix<ValueType_, Size_>::operator*=(const DiagonalMatrix& mat)
{
  // element-wise multiplication of the elements on both diagonals
  for (auto idx = 0; idx < Size_; ++idx)
  {
    _data.at_unsafe(idx) *= mat.at_unsafe(idx);
  }
}

template <typename ValueType_, sint32 Size_>
inline void DiagonalMatrix<ValueType_, Size_>::operator*=(const ValueType_ scalar)
{
  // element-wise multiplication of the elements on both diagonals
  for (auto idx = 0; idx < Size_; ++idx)
  {
    _data.at_unsafe(idx) *= scalar;
  }
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::inverse() const -> DiagonalMatrix
{
  DiagonalMatrix tmp{*this};
  tmp.inverse();
  return tmp;
}

template <typename ValueType_, sint32 Size_>
inline void DiagonalMatrix<ValueType_, Size_>::inverse()
{
  for (sint32 idx = 0; idx < Size_; ++idx)
  {
    assert((static_cast<ValueType_>(0) < this->at_unsafe(idx)) && "inverse not possible");
    _data.at_unsafe(idx) = static_cast<ValueType_>(1) / _data.at_unsafe(idx);
  }
}

template <typename ValueType_, sint32 Size_>
inline auto DiagonalMatrix<ValueType_, Size_>::isPositiveDefinite() const -> bool
{
  auto isValid = true;
  for (auto idx = 0; idx < Size_; ++idx)
  {
    isValid = isValid && (static_cast<ValueType_>(0) < _data.at_unsafe(idx));
  }
  return isValid;
}


// ------ non-member functions ---------------------------------------------------------------------------------------------------

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
auto operator*(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& mat,
               const DiagonalMatrix<ValueType_, Cols_>&             diag) -> Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>
{
  // each column is multiplied by the corresponding diagonal column element
  auto result{mat};
  for (auto col = 0; col < Cols_; ++col)
  {
    for (auto row = 0; row < Rows_; ++row)
    {
      result.at_unsafe(row, col) *= diag.at_unsafe(col);
    }
  }
  return result;
}

} // namespace math
} // namespace tracking

#endif // BA34774A_D1AA_44D4_BAD2_1845716F4E58
