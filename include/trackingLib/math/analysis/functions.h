#ifndef B8E086B0_F27F_43B8_9748_96B243918772
#define B8E086B0_F27F_43B8_9748_96B243918772

/// \file functions.h
/// \brief Mathematical functions for compile-time computations
///
/// This file provides mathematical functions that can be evaluated at compile time.
/// The implementation uses template metaprogramming to compute results during
/// compilation, avoiding runtime overhead for constant expressions.
///
/// \note All functions are constexpr and can be used in template parameters
///       and other compile-time contexts.

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace detail
{
/// \brief Helper struct for compile-time power computation
///
/// This template metaprogramming helper recursively computes x^N at compile time.
/// The recursion is unrolled by the compiler, resulting in zero runtime overhead.
///
/// \tparam T The value type (must support multiplication)
/// \tparam N The exponent (must be a compile-time constant)
template <class T, int N>
struct helper
{
  /// \brief Compute x^N recursively
  /// \param x The base value
  /// \return x^N
  static constexpr T pow(const T x) { return helper<T, N - 1>::pow(x) * x; }
};

/// \brief Specialization for N=1 (base case)
/// \tparam T The value type
template <class T>
struct helper<T, 1>
{
  /// \brief Return x^1 = x
  /// \param x The base value
  /// \return x
  static constexpr T pow(const T x) { return x; }
};

/// \brief Specialization for N=0 (base case)
/// \tparam T The value type
template <class T>
struct helper<T, 0>
{
  /// \brief Return x^0 = 1 (mathematical convention)
  /// \return 1
  static constexpr T pow(const T) { return 1; }
};
} // namespace detail

/// \brief Compile-time power function
///
/// Computes x^N at compile time using template metaprogramming.
/// This function has zero runtime overhead when used with compile-time constants.
///
/// \tparam N The exponent (must be a compile-time constant)
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
T constexpr pow(T const x)
{
  return detail::helper<T, N>::pow(x);
}

} // namespace math
} // namespace tracking

#endif // B8E086B0_F27F_43B8_9748_96B243918772
