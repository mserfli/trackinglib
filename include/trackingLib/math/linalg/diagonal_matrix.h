#ifndef EDCA948E_6A98_4AF3_8A01_916736E1577B
#define EDCA948E_6A98_4AF3_8A01_916736E1577B

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/errors.h"
#include "math/linalg/matrix_io.h"
#include <initializer_list>

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix;

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix;

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix;

template <typename ValueType_, sint32 Size_>
class Vector;

// TODO(matthias): add interface contract

/// \brief A diagonal matrix that stores only the diagonal elements for memory efficiency.
///
/// This class represents diagonal matrices where only the diagonal elements are stored
/// and manipulated. All off-diagonal elements are implicitly zero. Provides optimized
/// operations for diagonal-specific computations like inversion and multiplication.
///
/// \tparam ValueType_ The data type of diagonal elements (e.g., float32, float64)
/// \tparam Size_ The dimension of the diagonal matrix (compile-time constant)
///
/// \note Memory efficient: stores only Size_ elements instead of Size_²
/// \note All operations are O(Size_) instead of O(Size_²) for general matrices
///
/// \see SquareMatrix for general square matrix operations
/// \see TriangularMatrix for triangular matrix operations
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL
{
public:
  // rule of 5 declarations
  DiagonalMatrix()                                             = default;
  DiagonalMatrix(const DiagonalMatrix& other)                  = default;
  DiagonalMatrix(DiagonalMatrix&&) noexcept                    = default;
  auto operator=(const DiagonalMatrix&) -> DiagonalMatrix&     = default;
  auto operator=(DiagonalMatrix&&) noexcept -> DiagonalMatrix& = default;

  /// \brief Construct an identity diagonal matrix.
  ///
  /// Creates a diagonal matrix with ones on the diagonal (identity matrix).
  ///
  /// \return DiagonalMatrix An identity matrix with ones on the diagonal
  ///
  /// \note This is equivalent to a diagonal matrix with all diagonal elements equal to 1
  [[nodiscard]] static auto Identity() -> DiagonalMatrix;

  /// \brief Set the diagonal matrix to identity in-place.
  ///
  /// Modifies the diagonal elements to all be 1, creating an identity matrix.
  ///
  /// \note This operation modifies the matrix in-place and does not change its size
  void setIdentity();

  /// \brief Creates a DiagonalMatrix from a flat initializer list
  ///
  /// This function creates a diagonal matrix where the diagonal elements are taken from
  /// a flat initializer list. The list size must exactly match the matrix dimension.
  ///
  /// \param[in] list Initializer list containing the diagonal values
  /// \return DiagonalMatrix with the specified diagonal elements
  /// \note The list size must equal Size_, otherwise assertion fails
  [[nodiscard]] static auto FromList(const std::initializer_list<ValueType_>& list) -> DiagonalMatrix;

  /// \brief Creates a DiagonalMatrix from the diagonal of a nested initializer list
  ///
  /// This function creates a diagonal matrix by extracting the diagonal elements from
  /// a nested initializer list representing a full matrix. Only the diagonal elements
  /// (where row index equals column index) are used.
  ///
  /// \param[in] list Nested initializer list representing a square matrix
  /// \return DiagonalMatrix containing the diagonal elements from the input list
  /// \note The outer list size must equal Size_, and each inner list size must equal Size_
  [[nodiscard]] static auto FromList(const std::initializer_list<std::initializer_list<ValueType_>>& list) -> DiagonalMatrix;

  /// \brief Set a diagonal block matrix at given position
  /// \tparam SrcSize_    Size_ of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcIdxBeg  Begin diagonal element index in source
  /// \tparam DstIdxBeg  Begin diagonal element index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize_, sint32 SrcCount_, sint32 SrcIdxBeg_, sint32 DstIdxBeg_>
  void setBlock(const DiagonalMatrix<ValueType_, SrcSize_>& block);

  /// \brief Multiplication with generic matrix: D * Matrix
  /// \tparam Cols_
  /// \tparam IsRowMajor_
  /// \param[in] mat
  /// \return Matrix<ValueType_, Size_, Cols_, IsRowMajor_>
  template <sint32 Cols_, bool IsRowMajor_>
  [[nodiscard]] auto operator*(const Matrix<ValueType_, Size_, Cols_, IsRowMajor_>& mat) const
      -> Matrix<ValueType_, Size_, Cols_, IsRowMajor_>;

  /// \brief Multiplication with triangular matrix: D * Matrix
  /// \tparam isLower_
  /// \tparam isRowMajor_
  /// \param[in] mat  A triangular matrix
  /// \return TriangularMatrix<ValueType_, Size_, isLower_, isRowMajor_>
  template <bool IsLower_, bool IsRowMajor_>
  [[nodiscard]] auto operator*(const TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>& mat) const
      -> TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>;

  /// \brief Multiplication with diagonal matrix: D * Matrix
  /// \param[in] mat  A diagonal matrix
  /// \return DiagonalMatrix<ValueType_, Size_>
  [[nodiscard]] auto operator*(const DiagonalMatrix& mat) const -> DiagonalMatrix;

  /// \brief Multiplication with scalar: D * scalar
  /// \param[in] scalar  A scalar value
  /// \return DiagonalMatrix<ValueType_, Size_>
  [[nodiscard]] auto operator*(const ValueType_ scalar) const -> DiagonalMatrix;

  /// \brief Inplace Multiplication with diagonal matrix: D * Matrix
  /// \param[in] mat  A diagonal matrix
  void operator*=(const DiagonalMatrix& mat);

  /// \brief Inplace Multiplication with scalar: D * scalar
  /// \param[in] scalar  A scalar value
  void operator*=(const ValueType_ scalar);

  /// \brief Element access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return tl::expected<std::reference_wrapper<ValueType_>, Errors>   either the reference at (idx) or an Error descriptor
  [[nodiscard]] auto operator[](const sint32 idx) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>
  { // implemented here to solve cyclic includes
    if (!(idx >= 0 && idx < Size_))
    {
      return tl::unexpected<Errors>{Errors::invalid_access_idx};
    }
    return at_unsafe(idx);
  }

  /// \brief Element read-only access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return tl::expected<ValueType_, Errors>   either the value at (idx) or an Error descriptor
  [[nodiscard]] auto operator[](const sint32 idx) const -> tl::expected<ValueType_, Errors>
  { // implemented here to solve cyclic includes
    if (!(idx >= 0 && idx < Size_))
    {
      return tl::unexpected<Errors>{Errors::invalid_access_idx};
    }
    return at_unsafe(idx);
  }

  /// \brief Compute the inverse of the diagonal matrix.
  ///
  /// Calculates the inverse by taking the reciprocal of each diagonal element.
  /// The result is also a diagonal matrix.
  ///
  /// \return DiagonalMatrix The inverse matrix such that D * D^(-1) = I
  ///
  /// \warning Fails if any diagonal element is zero (singular matrix)
  /// \note O(Size_) complexity, very efficient for diagonal matrices
  [[nodiscard]] auto inverse() const -> DiagonalMatrix;

  /// \brief Compute the inverse in-place.
  ///
  /// Modifies this matrix to contain its inverse by taking reciprocals of diagonal elements.
  ///
  /// \warning Fails if any diagonal element is zero (singular matrix)
  /// \note More memory efficient than the const version for large matrices
  void inverse();

  /// \brief Calculate the trace of the diagonal matrix.
  ///
  /// Computes the sum of all diagonal elements of the matrix.
  /// The trace is defined as the sum of elements A_ii for i = 1 to n.
  ///
  /// \return ValueType_ The trace of the matrix(sum of diagonal elements)
  [[nodiscard]] auto trace() const -> ValueType_;

  /// \brief Calculate the determinant of the square matrix.
  ///
  /// Computes the determinant as the product of the diagonal elements.
  ///
  /// \return ValueType_ The determinant of the matrix
  /// \note Time complexity: O(n) where n is the matrix dimension
  /// \note For singular matrices, the determinant will be zero or very close to zero
  [[nodiscard]] auto determinant() const -> ValueType_;

  /// \brief Check if the diagonal matrix is positive definite.
  ///
  /// A diagonal matrix is positive definite if all diagonal elements are positive.
  ///
  /// \return true if all diagonal elements are > 0, false otherwise
  ///
  /// \note For diagonal matrices, positive definiteness is equivalent to all elements > 0
  [[nodiscard]] auto isPositiveDefinite() const -> bool;

  /// \brief Check if the diagonal matrix is positive semi-definite.
  ///
  /// A diagonal matrix is positive semi-definite if all diagonal elements are non-negative.
  ///
  /// \return true if all diagonal elements are >= 0, false otherwise
  ///
  /// \note For diagonal matrices, positive semi-definiteness means all elements >= 0
  [[nodiscard]] auto isPositiveSemiDefinite() const -> bool;

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_  Scalar vector value
  [[nodiscard]] auto at_unsafe(sint32 idx) const -> ValueType_ { return _data.at_unsafe(idx); }

  /// \brief Element access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_&  Reference to the scalar vector value
  [[nodiscard]] auto at_unsafe(sint32 idx) -> ValueType_& { return _data.at_unsafe(idx); }
  // <---

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  Vector<ValueType_, Size_> _data{};
};

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
[[nodiscard]] auto operator*(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& mat,
                             const DiagonalMatrix<ValueType_, Cols_>& diag) -> Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>;

} // namespace math
} // namespace tracking

#endif // EDCA948E_6A98_4AF3_8A01_916736E1577B
