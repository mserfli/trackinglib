#ifndef EDCA948E_6A98_4AF3_8A01_916736E1577B
#define EDCA948E_6A98_4AF3_8A01_916736E1577B

#include "base/first_include.h"
#include <initializer_list>

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Rows, sint32 Cols>
class Matrix; // LCOV_EXCL_LINE

template <typename FloatType, sint32 Size>
class SquareMatrix; // LCOV_EXCL_LINE

template <typename FloatType, sint32 Size, bool isLower>
class TriangularMatrix; // LCOV_EXCL_LINE

template <typename FloatType, sint32 Size>
class Vector; // LCOV_EXCL_LINE

// TODO(matthias): add interface contract
template <typename FloatType, sint32 Size>
class DiagonalMatrix TEST_REMOVE_FINAL
{
public:
  // rule of 5 declarations
  DiagonalMatrix()                            = default;
  DiagonalMatrix(const DiagonalMatrix& other) = default;
  DiagonalMatrix(DiagonalMatrix&&) noexcept   = default;
  auto operator=(const DiagonalMatrix&) -> DiagonalMatrix& = default;
  auto operator=(DiagonalMatrix&&) noexcept -> DiagonalMatrix& = default;

  /// \brief Construct a new Diagonal Matrix object
  /// \param[in] other
  DiagonalMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  DiagonalMatrix(const std::initializer_list<FloatType>& list);

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  DiagonalMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  /// \brief Set internal matrix to the Identity matrix
  void setIdentity();

  /// \brief Construct an Identity matrix
  /// \return DiagonalMatrix  Resulting identity matrix
  static auto Identity() -> DiagonalMatrix;

  /// \brief Set a diagonal block matrix at given position
  /// \tparam SrcSize    Size of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcIdxBeg  Begin diagonal element index in source
  /// \tparam DstIdxBeg  Begin diagonal element index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount, sint32 SrcIdxBeg, sint32 DstIdxBeg>
  void setBlock(const DiagonalMatrix<FloatType, SrcSize>& block);

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  auto operator=(const std::initializer_list<FloatType>& list) -> DiagonalMatrix&; // TODO(matthias): do we really need this

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  auto operator=(const std::initializer_list<std::initializer_list<FloatType>>& list)
      -> DiagonalMatrix&; // TODO(matthias): do we really need this

  /// \brief Multiplication with generic matrix: D * Matrix
  /// \tparam Cols
  /// \param[in] mat
  /// \return Matrix<FloatType, Size, Cols>
  template <sint32 Cols>
  auto operator*(const Matrix<FloatType, Size, Cols>& mat) const -> Matrix<FloatType, Size, Cols>;

  /// \brief Multiplication with triangular matrix: D * Matrix
  /// \tparam isLower
  /// \param[in] mat  A triangular matrix
  /// \return TriangularMatrix<FloatType, Size, isLower>
  template <bool isLower>
  auto operator*(const TriangularMatrix<FloatType, Size, isLower>& mat) const -> TriangularMatrix<FloatType, Size, isLower>;

  /// \brief Multiplication with diagonal matrix: D * Matrix
  /// \param[in] mat  A diagonal matrix
  /// \return DiagonalMatrix<FloatType, Size>
  auto operator*(const DiagonalMatrix& mat) const -> DiagonalMatrix;

  /// \brief Multiplication with scalar: D * scalar
  /// \param[in] scalar  A scalar value
  /// \return DiagonalMatrix<FloatType, Size>
  auto operator*(const FloatType scalar) const -> DiagonalMatrix;

  /// \brief Inplace Multiplication with diagonal matrix: D * Matrix
  /// \param[in] mat  A diagonal matrix
  /// \return DiagonalMatrix<FloatType, Size>&
  auto operator*=(const DiagonalMatrix& mat) -> DiagonalMatrix&;

  /// \brief Inplace Multiplication with scalar: D * scalar
  /// \param[in] scalar  A scalar value
  /// \return DiagonalMatrix<FloatType, Size>&
  auto operator*=(const FloatType scalar) -> DiagonalMatrix&;

  /// \brief Element access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return FloatType&  Reference to the scalar diagonal value
  auto operator[](const sint32 idx) -> FloatType&
  { // implemented here to solve cyclic includes
    assert(((0 <= idx) && (idx < Size)) && "Index out of bounds");
    return _data[idx];
  }

  /// \brief Element read-only access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return FloatType  Scalar diagonal value
  auto operator[](const sint32 idx) const -> FloatType
  { // implemented here to solve cyclic includes
    assert(((0 <= idx) && (idx < Size)) && "Index out of bounds");
    return _data[idx];
  }

  /// \brief Calculates the inverse
  /// \return DiagonalMatrix<FloatType, Size>
  auto inverse() const -> DiagonalMatrix;

  /// \brief Calculates the inverse inplace
  void inverse();

  /// \brief Checks for positive definite condition of the diagonal matrix (all elements > 0)
  [[nodiscard]] auto isPositiveDefinite() const -> bool;

  /// \brief Print the matrix
  void print() const;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround to keep following idententation
  // clang-format on
  Vector<FloatType, Size> _data{};
};

template <typename FloatType, sint32 Rows, sint32 Cols>
auto operator*(const Matrix<FloatType, Rows, Cols>& mat, const DiagonalMatrix<FloatType, Cols>& diag)
    -> Matrix<FloatType, Rows, Cols>;

} // namespace math
} // namespace tracking

#endif // EDCA948E_6A98_4AF3_8A01_916736E1577B
