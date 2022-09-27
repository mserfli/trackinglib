#ifndef EDCA948E_6A98_4AF3_8A01_916736E1577B
#define EDCA948E_6A98_4AF3_8A01_916736E1577B

#include "math/linalg/square_matrix.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
class DiagonalMatrix: public SquareMatrix<FloatType, Size>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using SquareMatrix<FloatType, Size>::SquareMatrix;

  /// \brief Construct a new Diagonal Matrix object
  /// \param[in] other
  DiagonalMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  DiagonalMatrix(const std::initializer_list<FloatType>& list);

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  DiagonalMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  auto operator=(const std::initializer_list<FloatType>& list) -> DiagonalMatrix<FloatType, Size>&;

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  auto operator=(const std::initializer_list<std::initializer_list<FloatType>>& list) -> DiagonalMatrix<FloatType, Size>&;

  /// \brief Element access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return FloatType&  Reference to the scalar diagonal value
  auto operator[](const sint32 idx) -> FloatType&;

  /// \brief Element read-only access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return FloatType  Scalar diagonal value
  auto operator[](const sint32 idx) const -> FloatType;

private:
  /// \brief hide inherited operator() to prevent accessing off-diagonal elements
  using SquareMatrix<FloatType, Size>::operator();
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
inline auto DiagonalMatrix<FloatType, Size>::operator=(const std::initializer_list<FloatType>& list)
    -> DiagonalMatrix<FloatType, Size>&
{
  *this = DiagonalMatrix<FloatType, Size>(list);
  return *this;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator=(const std::initializer_list<std::initializer_list<FloatType>>& list)
    -> DiagonalMatrix<FloatType, Size>&
{
  *this = DiagonalMatrix<FloatType, Size>(list);
  return *this;
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator[](const sint32 idx) -> FloatType&
{
  return this->operator()(idx, idx);
}

template <typename FloatType, sint32 Size>
inline auto DiagonalMatrix<FloatType, Size>::operator[](const sint32 idx) const -> FloatType
{
  return this->operator()(idx, idx);
}

} // namespace math
} // namespace tracking

#endif // EDCA948E_6A98_4AF3_8A01_916736E1577B
