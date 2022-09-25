#ifndef EDCA948E_6A98_4AF3_8A01_916736E1577B
#define EDCA948E_6A98_4AF3_8A01_916736E1577B

#include "base/atomic_types.h"
#include "base/square_matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size>
class DiagonalMatrix: public SquareMatrix<FloatType, Size>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using SquareMatrix<FloatType, Size>::SquareMatrix;

  DiagonalMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  DiagonalMatrix(const std::initializer_list<FloatType>& list);

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  DiagonalMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  auto operator[](const sint32 idx) -> FloatType&;
};

template <typename FloatType, sint32 Size>
DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{}
{
  // copy diagonal elements from other
  for (sint32 row = 0; row < Size; ++row)
  {
    this->operator()(row, row) = other(row, row);
  }
}

template <typename FloatType, sint32 Size>
DiagonalMatrix<FloatType, Size>::DiagonalMatrix(const std::initializer_list<FloatType>& list)
    : SquareMatrix<FloatType, Size>{}
{
  static_assert(list.size() == Size, "Mismatching size of intializer list");

  // fill diagonal elements
  for (sint32 idx = 0; idx < Size; ++idx)
  {
    this->operator()(idx, idx) = list[idx];
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
inline auto DiagonalMatrix<FloatType, Size>::operator[](const sint32 idx) -> FloatType&
{
  return this->operator()(idx, idx);
}

} // namespace base
} // namespace tracking

#endif // EDCA948E_6A98_4AF3_8A01_916736E1577B
