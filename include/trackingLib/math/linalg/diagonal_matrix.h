#ifndef EDCA948E_6A98_4AF3_8A01_916736E1577B
#define EDCA948E_6A98_4AF3_8A01_916736E1577B

#include "base/first_include.h"
#include "math/linalg/square_matrix.hpp"
#include <initializer_list>

namespace tracking
{
namespace math
{
// TODO(matthias): add interface contract
template <typename FloatType, sint32 Size>
class DiagonalMatrix: public SquareMatrix<FloatType, Size>
{
public:
  // rule of 5 declarations
  DiagonalMatrix()                            = default;
  DiagonalMatrix(const DiagonalMatrix& other) = default;
  DiagonalMatrix(DiagonalMatrix&&) noexcept   = default;
  auto operator=(const DiagonalMatrix&) -> DiagonalMatrix& = default;
  auto operator=(DiagonalMatrix&&) noexcept -> DiagonalMatrix& = default;
  virtual ~DiagonalMatrix()                                    = default;

  /// \brief Construct a new Diagonal Matrix object
  /// \param[in] other
  DiagonalMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  DiagonalMatrix(const std::initializer_list<FloatType>& list);

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  DiagonalMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  /// \brief Set a diagonal block matrix at given position
  /// \tparam SrcSize    Size of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcIdxBeg  Begin diagonal element index in source
  /// \tparam DstIdxBeg  Begin diagonal element index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount, sint32 SrcIdxBeg, sint32 DstIdxBeg>
  void setBlock(const DiagonalMatrix<FloatType, SrcSize>& block);

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  auto operator=(const std::initializer_list<FloatType>& list) -> DiagonalMatrix&;

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  auto operator=(const std::initializer_list<std::initializer_list<FloatType>>& list) -> DiagonalMatrix&;

  template <sint32 Cols>
  auto operator*(const Matrix<FloatType, Size, Cols>& mat) -> Matrix<FloatType, Size, Cols>
  {
    auto temp(mat);
    for (auto row = 0; row < Size; ++row)
    {
      for (auto col = 0; col < Cols; ++col)
      {
        temp(row, col) *= this->operator[](col);
      }
    }
    return temp;
  }

  /// \brief Element access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return FloatType&  Reference to the scalar diagonal value
  auto operator[](const sint32 idx) -> FloatType&;

  /// \brief Element read-only access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return FloatType  Scalar diagonal value
  auto operator[](const sint32 idx) const -> FloatType;

  /// \brief Calculates the inverse
  /// \return DiagonalMatrix<FloatType, Size>
  auto inverse() const -> DiagonalMatrix;

  /// \brief Calculates the inverse inplace
  void inverse();

  /// \brief Checks for positive definite condition of the diagonal matrix (all elements > 0)
  auto isPositiveDefinite() const -> bool;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround to keep following idententation
  // clang-format on
  /// \brief hide inherited operator() to prevent accessing off-diagonal elements
  using SquareMatrix<FloatType, Size>::operator();

  /// \brief hide inherited setBlock function
  using SquareMatrix<FloatType, Size>::setBlock;
};

template <typename FloatType, sint32 Size>
DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{}
{
  // copy diagonal elements from other
  for (sint32 idx = 0; idx < Size; ++idx)
  {
    SquareMatrix<FloatType, Size>::operator()(idx, idx) = other(idx, idx);
  }
}

template <typename FloatType, sint32 Size>
DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const std::initializer_list<FloatType>& list)
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
DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list)
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
void DiagonalMatrix<FloatType, Size>::setBlock(const DiagonalMatrix<FloatType, SrcSize>& block)
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
auto DiagonalMatrix<FloatType, Size>::isPositiveDefinite() const -> bool
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

#endif // EDCA948E_6A98_4AF3_8A01_916736E1577B
