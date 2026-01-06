#ifndef B8E086B0_F27F_43B8_9748_96B243918772
#define B8E086B0_F27F_43B8_9748_96B243918772

/// \file functions.h
/// \brief Mathematical functions for compile-time computations
///
/// This file provides mathematical functions that can be evaluated at compile time.
/// The implementation uses modern C++17 constexpr functions for better readability
/// and maintainability compared to template metaprogramming approaches.
///
/// \note All functions are constexpr and can be used in template parameters
///       and other compile-time contexts.

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{
/// \brief Compile-time power function
///
/// Computes x^N at compile time using a simple iterative approach.
/// This modern C++17 implementation replaces the previous template metaprogramming
/// approach, providing better readability and maintainability while maintaining
/// zero runtime overhead when used with compile-time constants.
///
/// \tparam N The exponent (must be a compile-time constant, non-negative integer)
/// \tparam T The value type (must support multiplication)
/// \param x The base value
/// \return x raised to the power N
///
/// \note This function is constexpr and can be used in template parameters
/// \note Negative exponents are not supported (would require floating-point division)
/// \note For runtime exponents, use std::pow from cmath
///
/// Example usage:
/// \code{.cpp}
/// constexpr double result = tracking::math::pow<3>(2.0); // result = 8.0
/// constexpr int template_param = tracking::math::pow<4>(3); // can be used in templates
/// \endcode
template <int N, class T>
constexpr T pow(T const x)
{
  if constexpr (N == 0)
  {
    return 1; // x^0 = 1 (mathematical convention)
  }
  else if constexpr (N == 1)
  {
    return x; // x^1 = x
  }
  else
  {
    // Iterative approach for N >= 2
    T result = x;
    for (int i = 1; i < N; ++i)
    {
      result *= x;
    }
    return result;
  }
}

} // namespace math
} // namespace tracking

#endif // B8E086B0_F27F_43B8_9748_96B243918772
