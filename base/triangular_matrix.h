#ifndef BC7FD90F_FBB7_481C_89C4_89BEE41309C5
#define BC7FD90F_FBB7_481C_89C4_89BEE41309C5

#include "base/square_matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size, bool isLower>
class TriangularMatrix: public SquareMatrix<FloatType, Size>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using SquareMatrix<FloatType, Size>::SquareMatrix;

  TriangularMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new TriangularMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  TriangularMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);
};

template <typename FloatType, sint32 Size, bool isLower>
TriangularMatrix<FloatType, Size, isLower>::TriangularMatrix(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{}
{
  // copy triangular elements from other
  for (sint32 row = 0; row < Size; ++row)
  {
    this->operator()(row, row) = other(row, row);
    for (sint32 col = row + 1; col < Size; ++col)
    {
      const sint32 rowIdx = isLower ? col : row;
      const sint32 colIdx = isLower ? row : col;

      this->operator()(rowIdx, colIdx) = other(rowIdx, colIdx);
    }
  }
}

template <typename FloatType, sint32 Size, bool isLower>
TriangularMatrix<FloatType, Size, isLower>::TriangularMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list)
    : SquareMatrix<FloatType, Size>{list}
{
  // zero anti-triangular elements
  for (sint32 row = 0; row < Size; ++row)
  {
    for (sint32 col = row + 1; col < Size; ++col)
    {
      const sint32 rowIdx = !isLower ? col : row;
      const sint32 colIdx = !isLower ? row : col;

      this->operator()(rowIdx, colIdx) = static_cast<FloatType>(0.0);
    }
  }
}

} // namespace base
} // namespace tracking
#endif // BC7FD90F_FBB7_481C_89C4_89BEE41309C5
