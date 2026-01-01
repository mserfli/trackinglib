#ifndef C63CC10A_92E8_4A79_8412_56D9CFCB8DB4
#define C63CC10A_92E8_4A79_8412_56D9CFCB8DB4

#include "base/first_include.h" // IWYU pragma: keep
#include <iomanip>
#include <ostream>
#include <type_traits>

namespace tracking
{
namespace math
{

// Forward declarations
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix;

// SFINAE helper to detect matrix-like types
template <typename T, typename = void>
struct is_matrix_like: std::false_type
{
};

template <typename T>
struct is_matrix_like<T,
                      std::void_t<decltype(std::declval<const T&>().at_unsafe(sint32{}, sint32{})),
                                  typename T::value_type,
                                  decltype(T::Rows),
                                  decltype(T::Cols)>>: std::true_type
{
};

template <typename T>
inline constexpr bool is_matrix_like_v = is_matrix_like<T>::value;

// Template operator<< for all matrix-like types
// Primary template for general matrix-like types
template <typename M>
auto operator<<(std::ostream& os, const M& matrix) -> std::enable_if_t<is_matrix_like_v<M>, std::ostream&>
{
  using ValueType = typename M::value_type;

  // Access elements via at_unsafe() - works regardless of internal memory layout
  // This is future-proof for packed triangular matrices
  for (sint32 row = 0; row < M::Rows; ++row)
  {
    for (sint32 col = 0; col < M::Cols; ++col)
    {
      if constexpr (std::is_floating_point_v<ValueType>)
      {
        os << std::fixed << std::setprecision(6) << std::showpos << std::setw(12) << matrix.at_unsafe(row, col);
      }
      else
      {
        os << std::setw(8) << matrix.at_unsafe(row, col);
      }

      if (col < M::Cols - 1)
      {
        os << ", ";
      }
    }
    os << "\n";
  }
  return os;
}

// Specialization for DiagonalMatrix which has different at_unsafe signature
template <typename ValueType, sint32 Size>
std::ostream& operator<<(std::ostream& os, const tracking::math::DiagonalMatrix<ValueType, Size>& matrix)
{
  using ValueType_ = ValueType;

  for (sint32 row = 0; row < Size; ++row)
  {
    for (sint32 col = 0; col < Size; ++col)
    {
      if constexpr (std::is_floating_point_v<ValueType_>)
      {
        os << std::fixed << std::setprecision(6) << std::showpos << std::setw(12);
      }
      else
      {
        os << std::setw(8);
      }

      // For diagonal matrix, only diagonal elements are stored
      if (row == col)
      {
        os << matrix.at_unsafe(row);
      }
      else
      {
        os << static_cast<ValueType_>(0);
      }

      if (col < Size - 1)
      {
        os << ", ";
      }
    }
    os << "\n";
  }
  return os;
}
} // namespace math
} // namespace tracking

#endif // C63CC10A_92E8_4A79_8412_56D9CFCB8DB4
