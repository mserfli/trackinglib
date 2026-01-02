#ifndef B03BEBEF_0868_4245_A0B5_AD255C4514FE
#define B03BEBEF_0868_4245_A0B5_AD255C4514FE

/// \file errors.h
/// \brief Error types used throughout the math library
///
/// This file defines the error enumeration used by the math library's
/// error handling system. These errors are returned via tl::expected<T, Errors>
/// to provide safe error propagation without exceptions.
///
/// \note All error values are used in conjunction with tl::expected to
///       indicate operation failures in matrix and vector operations.

namespace tracking
{
namespace math
{

/// \brief Enumeration of possible error conditions in math operations
///
/// This enum defines all possible error conditions that can occur during
/// matrix and vector operations. Each error corresponds to a specific
/// failure condition that prevents an operation from completing successfully.
///
/// \note Used with tl::expected<T, Errors> for error handling
/// \see tl::expected for the error handling pattern used
enum class Errors
{
  invalid_access_row,           ///< Row index is out of bounds (e.g., accessing row 5 in a 3x3 matrix)
  invalid_access_col,           ///< Column index is out of bounds (e.g., accessing column 5 in a 3x3 matrix)
  invalid_access_idx,           ///< Linear index is out of bounds (e.g., accessing element 10 in a 3x3 matrix)
  divide_by_zero,               ///< Division by zero attempted (e.g., 9 / 0)
  not_integer_division,         ///< Result of integer division is not an integer (e.g., 5 / 2 = 2.5)
  integer_divide_overflows,     ///< Integer division would overflow (e.g., INT_MIN / -1)
  matrix_not_symmetric,         ///< Matrix operation requires symmetric matrix but input is not symmetric
  matrix_not_positive_definite, ///< Matrix decomposition requires positive definite matrix but input is not
};

} // namespace math
} // namespace tracking

#endif // B03BEBEF_0868_4245_A0B5_AD255C4514FE
