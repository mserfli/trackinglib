#ifndef BC7FD90F_FBB7_481C_89C4_89BEE41309C5
#define BC7FD90F_FBB7_481C_89C4_89BEE41309C5

#include "base/first_include.h"
#include "math/linalg/square_matrix.h"
#include <initializer_list>

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Rows, sint32 Cols>
class Matrix; // LCOV_EXCL_LINE

template <typename FloatType, sint32 Size>
class DiagonalMatrix; // LCOV_EXCL_LINE

// TODO(matthias): add interface contract
// TODO(matthias): speedup transpose by storing the current transpose status and swap col/row access
// TODO(matthias): use own memory optimized to required number of elements
template <typename FloatType, sint32 Size, bool isLower>
class TriangularMatrix TEST_REMOVE_FINAL: public SquareMatrix<FloatType, Size>
{
public:
  // rule of 5 declarations
  TriangularMatrix()                              = default;
  TriangularMatrix(const TriangularMatrix& other) = default;
  TriangularMatrix(TriangularMatrix&&) noexcept   = default;
  auto operator=(const TriangularMatrix&) -> TriangularMatrix& = default;
  auto operator=(TriangularMatrix&&) noexcept -> TriangularMatrix& = default;
  ~TriangularMatrix()                                              = default;

  /// \brief Construct a new Triangular Matrix object
  /// \param[in] other
  TriangularMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new TriangularMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  TriangularMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  /// \brief Set a lower/upper triangular block matrix at given position
  /// \tparam SrcSize    Size of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcRowBeg  Begin row index in source
  /// \tparam SrcColBeg  Begin col index in source
  /// \tparam DstRowBeg  Begin row index in dest
  /// \tparam DstColBeg  Begin col index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount, sint32 SrcRowBeg, sint32 SrcColBeg, sint32 DstRowBeg, sint32 DstColBeg>
  void setBlock(const TriangularMatrix<FloatType, SrcSize, isLower>& block);

  /// \brief Multiplication with generic matrix: Tria * Matrix
  /// \tparam Cols
  /// \param[in] mat
  /// \return Matrix<FloatType, Size, Cols>
  template <sint32 Cols>
  auto operator*(const Matrix<FloatType, Size, Cols>& mat) const -> Matrix<FloatType, Size, Cols>;

  /// \brief Multiplication with triangular matrix: Tria * Matrix
  /// \param[in] mat  A triangular matrix
  /// \return TriangularMatrix<FloatType, Size, isLower>
  auto operator*(const TriangularMatrix<FloatType, Size, isLower>& mat) const -> TriangularMatrix;

  /// \brief Multiplication with triangular matrix: Tria * Matrix
  /// \param[in] mat  A triangular matrix
  /// \return SquareMatrix<FloatType, Size>
  auto operator*(const TriangularMatrix<FloatType, Size, !isLower>& mat) const -> SquareMatrix<FloatType, Size>;

  /// \brief Multiplication with diagonal matrix: Tria * Matrix
  /// \param[in] diag  A diagonal matrix
  /// \return TriangularMatrix<FloatType, Size, isLower>
  auto operator*(const DiagonalMatrix<FloatType, Size>& diag) const -> TriangularMatrix;

  /// \brief Multiplication with scalar: Tria * scalar
  /// \param[in] scalar  A scalar value
  /// \return TriangularMatrix<FloatType, Size, isLower>
  auto operator*(const FloatType scalar) const -> TriangularMatrix;

  /// \brief Inplace Multiplication with scalar: Tria * scalar
  /// \param[in] scalar  A scalar value
  /// \return TriangularMatrix<FloatType, Size, isLower>&
  auto operator*=(const FloatType scalar) -> TriangularMatrix&;

  /// \brief Element read-only access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType  scalar triangular value
  auto operator()(sint32 row, sint32 col) const -> FloatType;

  /// \brief Element access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType&  Reference to the scalar triangular value
  auto operator()(sint32 row, sint32 col) -> FloatType&;

  /// \brief Calculate the transposed matrix
  /// \return TriangularMatrix<FloatType, Size, !isLower>
  auto transpose() const -> TriangularMatrix<FloatType, Size, !isLower>;

  /// \brief Solver for A*x=b based on Cholesky decomposition of A
  /// \tparam Cols  Number of columns in the rhs variable b
  /// \param[in] b  Matrix describing the rhs of the equation A*x=b
  /// \return Matrix<FloatType, Size, Cols> describing x
  template <sint32 Cols>
  auto solve(const Matrix<FloatType, Size, Cols>& b) const -> Matrix<FloatType, Size, Cols>;

  /// \brief Calculates the inverse of the underlying matrix
  /// \return TriangularMatrix
  auto inverse() const -> TriangularMatrix;
  // TODO(matthias): UnitUpper inplace inverse, Grewal Table 6.7 p.235

  /// \brief Checks for Unit Upper condition
  /// \return true
  [[nodiscard]] auto isUnitUpperTriangular() const -> bool;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround to keep following idententation
  // clang-format on
  /// \brief hide inherited transpose function
  using SquareMatrix<FloatType, Size>::transpose;

  /// \brief hide inherited operator() to prevent accessing off-triangular elements
  using SquareMatrix<FloatType, Size>::operator();
};

} // namespace math
} // namespace tracking

#endif // BC7FD90F_FBB7_481C_89C4_89BEE41309C5
