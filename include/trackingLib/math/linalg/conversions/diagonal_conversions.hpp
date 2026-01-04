#ifndef A000E27D_C91E_4768_A8FD_B292AB7B986A
#define A000E27D_C91E_4768_A8FD_B292AB7B986A

#include "conversions.h"
#include "math/linalg/diagonal_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"   // IWYU pragma: keep
#include <initializer_list>

namespace tracking
{
namespace math
{
namespace conversions
{

/// \brief Creates a DiagonalMatrix from a flat initializer list
///
/// This function creates a diagonal matrix where the diagonal elements are taken from
/// a flat initializer list. The list size must exactly match the matrix dimension.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The size of the diagonal matrix
/// \param[in] list Initializer list containing the diagonal values
/// \return DiagonalMatrix with the specified diagonal elements
/// \note The list size must equal Size_, otherwise assertion fails
/// \see DiagonalFromSquare() for creating from square matrices
/// \see DiagonalFromList() (overloaded) for nested list input
template <typename ValueType_, sint32 Size_>
inline auto DiagonalFromList(const std::initializer_list<ValueType_>& list) -> DiagonalMatrix<ValueType_, Size_>
{
  assert((list.size() == Size_) && "Mismatching size of intializer list");

  DiagonalMatrix<ValueType_, Size_> diag{};
  // fill diagonal elements
  sint32 idx = 0;
  for (auto val : list)
  {
    diag.at_unsafe(idx++) = val;
  }
  return diag;
}

/// \brief Creates a DiagonalMatrix from the diagonal of a nested initializer list
///
/// This function creates a diagonal matrix by extracting the diagonal elements from
/// a nested initializer list representing a full matrix. Only the diagonal elements
/// (where row index equals column index) are used.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The size of the diagonal matrix
/// \param[in] list Nested initializer list representing a square matrix
/// \return DiagonalMatrix containing the diagonal elements from the input list
/// \note The outer list size must equal Size_, and each inner list size must equal Size_
/// \see DiagonalFromSquare() for creating from square matrices
/// \see DiagonalFromList() (overloaded) for flat list input
template <typename ValueType_, sint32 Size_>
inline auto DiagonalFromList(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> DiagonalMatrix<ValueType_, Size_>
{
  assert(list.size() == Size_);
  assert(list.begin()->size() == Size_);

  DiagonalMatrix<ValueType_, Size_> diag{};
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

/// \brief Creates a DiagonalMatrix from the diagonal elements of a SquareMatrix
///
/// This function extracts the diagonal elements from a square matrix to create a diagonal matrix.
/// All off-diagonal elements are discarded, preserving only the main diagonal values.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The size of the square matrix and resulting diagonal matrix
/// \tparam IsRowMajor_ The storage layout of the source matrix
/// \param[in] mat The source square matrix
/// \return DiagonalMatrix containing the diagonal elements of the input matrix
/// \see DiagonalFromList() for creating diagonal matrices from initializer lists
/// \see SquareFromDiagonal() for the reverse conversion
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto DiagonalFromSquare(const SquareMatrix<ValueType_, Size_, IsRowMajor_>& mat) -> DiagonalMatrix<ValueType_, Size_>
{
  DiagonalMatrix<ValueType_, Size_> result;
  for (sint32 i = 0; i < Size_; ++i)
  {
    result.at_unsafe(i) = mat.at_unsafe(i, i);
  }
  return result;
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // A000E27D_C91E_4768_A8FD_B292AB7B986A
