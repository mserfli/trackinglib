/// \file matrix_io.h
/// \brief Stream output operators for all matrix types in the trackingLib math library.
///
/// This file provides template-based operator<< implementations for streaming matrix objects
/// to any std::ostream (cout, files, stringstream). The design uses SFINAE-based type detection
/// to work with all matrix-like types that provide the required interface.
///
/// Key features:
/// - Works with any std::ostream (cout, files, stringstream)
/// - Template-based implementation for compile-time type safety
/// - SFINAE-based type detection for C++17 compatibility
/// - Specialized formatting for DiagonalMatrix, TriangularMatrix, and Vector
/// - Future-proof design using at_unsafe() interface
///
/// \note This file replaces the previous print() methods across all matrix types,
///       providing idiomatic C++ stream output without code duplication.
///
/// \see matrix.h for the base Matrix class
/// \see diagonal_matrix.h for DiagonalMatrix specialization
/// \see vector.h for Vector specialization

#ifndef C63CC10A_92E8_4A79_8412_56D9CFCB8DB4
#define C63CC10A_92E8_4A79_8412_56D9CFCB8DB4

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/triangular_matrix.h"
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

template <typename ValueType_, sint32 Size_>
class Vector;

/// \brief SFINAE helper to detect matrix-like types at compile time.
///
/// This trait uses SFINAE (Substitution Failure is Not An Error) to detect whether
/// a type provides the interface expected of matrix-like objects. It checks for:
/// - An at_unsafe(sint32, sint32) method for element access
/// - A nested value_type typedef
/// - Static Rows and Cols constants
///
/// This allows the operator<< template to work with any matrix type that provides
/// the required interface, enabling compile-time polymorphism without inheritance.
///
/// \tparam T The type to check for matrix-like properties
/// \tparam Enable SFINAE parameter (automatically deduced)
///
/// \see operator<<(std::ostream&, const M&) for usage
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

/// \brief Convenience variable template for is_matrix_like trait.
///
/// Provides a more convenient syntax for checking if a type is matrix-like.
/// Equivalent to is_matrix_like<T>::value.
///
/// \tparam T The type to check
///
/// \see is_matrix_like
template <typename T>
inline constexpr bool is_matrix_like_v = is_matrix_like<T>::value;

/// \brief Template operator<< for streaming matrix-like objects to output streams.
///
/// This function provides idiomatic C++ stream output for all matrix types in the trackingLib
/// math library. It uses SFINAE to enable the operator only for types that provide the
/// required matrix interface.
///
/// The output format provides:
/// - Fixed-width columns for alignment
/// - 6 decimal places for floating-point types with sign display
/// - Comma-separated values within rows
/// - One row per line
///
/// \tparam M The matrix type (automatically deduced)
/// \param[in,out] os The output stream to write to
/// \param[in] matrix The matrix object to stream
/// \return Reference to the output stream for chaining
///
/// \note Uses at_unsafe() for element access, which is future-proof for packed storage
/// \note Formatting is optimized for readability in debug/console output
///
/// \see operator<<(std::ostream&, const DiagonalMatrix&) for DiagonalMatrix specialization
///
/// Usage example:
/// \code{.cpp}
/// Matrix<float32, 2, 3> mat = /* ... */;
/// std::cout << mat << std::endl;
/// // Output:
/// // +1.000000, +2.000000, +3.000000
/// // +4.000000, +5.000000, +6.000000
/// \endcode
template <typename M>
auto operator<<(std::ostream& os, const M& matrix) -> std::enable_if_t<is_matrix_like_v<M>, std::ostream&>
{
  using ValueType_ = typename M::value_type;

  // Access elements via at_unsafe() - works regardless of internal memory layout
  // This is future-proof for packed triangular matrices
  for (sint32 row = 0; row < M::Rows; ++row)
  {
    for (sint32 col = 0; col < M::Cols; ++col)
    {
      if constexpr (std::is_floating_point_v<ValueType_>)
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

/// \brief Specialization of operator<< for TriangularMatrix.
///
/// TriangularMatrix requires special handling because it doesn't allow to access
/// off-diagonal matrix elements
/// This specialization outputs the full matrix representation with zeros off-diagonal.
///
/// The output format matches the general matrix operator<< for consistency:
/// - Fixed-width columns for alignment
/// - 6 decimal places for floating-point types with sign display
/// - Triangular elements from stored values, off-diagonal as zero
///
/// \tparam ValueType_ The element type (float32, float64, etc.)
/// \tparam Size_ The matrix dimension (compile-time constant)
/// \tparam IsLower_ Whether the matrix is lower triangular
/// \tparam IsRowMajor_ Whether the matrix is stored in row-major order
/// \param[in,out] os The output stream to write to
/// \param[in] matrix The TriangularMatrix object to stream
/// \return Reference to the output stream for chaining
///
/// \note This specialization is necessary because TriangularMatrix::at_unsafe() has
///       restricted access to only triangular elements
///
/// \see TriangularMatrix for the matrix class
/// \see operator<<(std::ostream&, const M&) for general matrix streaming
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
std::ostream& operator<<(std::ostream&                                                                     os,
                         const tracking::math::TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>& matrix)
{
  for (sint32 row = 0; row < Size_; ++row)
  {
    for (sint32 col = 0; col < Size_; ++col)
    {
      if constexpr (std::is_floating_point_v<ValueType_>)
      {
        os << std::fixed << std::setprecision(6) << std::showpos << std::setw(12);
      }
      else
      {
        os << std::setw(8);
      }

      // For triangular matrix, only triangular elements are stored
      if ((IsLower_ && row >= col) || (!IsLower_ && row <= col))
      {
        os << matrix.at_unsafe(row, col);
      }
      else
      {
        os << static_cast<ValueType_>(0);
      }

      if (col < Size_ - 1)
      {
        os << ", ";
      }
    }
    os << "\n";
  }
  return os;
}

/// \brief Specialization of operator<< for DiagonalMatrix.
///
/// DiagonalMatrix requires special handling because it stores only diagonal elements
/// and has a different at_unsafe() signature (single index instead of row,col).
/// This specialization outputs the full matrix representation with zeros off-diagonal.
///
/// The output format matches the general matrix operator<< for consistency:
/// - Fixed-width columns for alignment
/// - 6 decimal places for floating-point types with sign display
/// - Diagonal elements from stored values, off-diagonal as zero
///
/// \tparam ValueType The element type (float32, float64, etc.)
/// \tparam Size The matrix dimension (compile-time constant)
/// \param[in,out] os The output stream to write to
/// \param[in] matrix The DiagonalMatrix object to stream
/// \return Reference to the output stream for chaining
///
/// \note This specialization is necessary because DiagonalMatrix::at_unsafe() takes
///       only one parameter (diagonal index) unlike general matrices
///
/// \see DiagonalMatrix for the matrix class
/// \see operator<<(std::ostream&, const M&) for general matrix streaming
///
/// Usage example:
/// \code{.cpp}
/// DiagonalMatrix<float32, 3> diag = /* diagonal [1, 2, 3] */;
/// std::cout << diag << std::endl;
/// // Output:
/// // +1.000000, +0.000000, +0.000000
/// // +0.000000, +2.000000, +0.000000
/// // +0.000000, +0.000000, +3.000000
/// \endcode
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

/// \brief Specialization of operator<< for Vector.
///
/// Vector requires special handling because it has a different at_unsafe() signature
/// (single index instead of row,col) and represents a column vector.
/// This specialization outputs the vector as a column matrix.
///
/// The output format matches the general matrix operator<< for consistency:
/// - Fixed-width columns for alignment
/// - 6 decimal places for floating-point types with sign display
/// - Vector elements as a single column
///
/// \tparam ValueType_ The element type (float32, float64, etc.)
/// \tparam Size_ The vector dimension (compile-time constant)
/// \param[in,out] os The output stream to write to
/// \param[in] vector The Vector object to stream
/// \return Reference to the output stream for chaining
///
/// \note This specialization is necessary because Vector::at_unsafe() takes
///       only one parameter (vector index) unlike general matrices
///
/// \see Vector for the vector class
/// \see operator<<(std::ostream&, const M&) for general matrix streaming
///
/// Usage example:
/// \code{.cpp}
/// Vector<float32, 3> vec = /* [1, 2, 3] */;
/// std::cout << vec << std::endl;
/// // Output:
/// // +1.000000
/// // +2.000000
/// // +3.000000
/// \endcode
template <typename ValueType_, sint32 Size_>
std::ostream& operator<<(std::ostream& os, const tracking::math::Vector<ValueType_, Size_>& vector)
{
  for (sint32 row = 0; row < Size_; ++row)
  {
    if constexpr (std::is_floating_point_v<ValueType_>)
    {
      os << std::fixed << std::setprecision(6) << std::showpos << std::setw(12) << vector.at_unsafe(row);
    }
    else
    {
      os << std::setw(8) << vector.at_unsafe(row);
    }
    os << "\n";
  }
  return os;
}

} // namespace math
} // namespace tracking

#endif // C63CC10A_92E8_4A79_8412_56D9CFCB8DB4
