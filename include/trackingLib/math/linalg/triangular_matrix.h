#ifndef BC7FD90F_FBB7_481C_89C4_89BEE41309C5
#define BC7FD90F_FBB7_481C_89C4_89BEE41309C5

#include "base/first_include.h"
#include "math/linalg/square_matrix.h"
#include <initializer_list>

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix; // LCOV_EXCL_LINE

// TODO(matthias): add interface contract
// TODO(matthias): speedup transpose by storing the current transpose status and swap col/row access
// TODO(matthias): use own memory optimized to required number of elements
template <typename ValueType_, sint32 Size_, bool IsLower_>
class TriangularMatrix TEST_REMOVE_FINAL: public SquareMatrix<ValueType_, Size_, true>
{
public:
  using SquareMatrix = SquareMatrix<ValueType_, Size_, true>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using SquareMatrix::SquareMatrix;

  /// \brief Construct a new Triangular Matrix object
  /// \param[in] other
  explicit TriangularMatrix(const SquareMatrix& other);

  /// \brief Construct a new TriangularMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  TriangularMatrix(const std::initializer_list<std::initializer_list<ValueType_>>& list);

  /// \brief Set a lower/upper triangular block matrix at given position
  /// \tparam SrcSize    Size of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcRowBeg  Begin row index in source
  /// \tparam SrcColBeg  Begin col index in source
  /// \tparam DstRowBeg  Begin row index in dest
  /// \tparam DstColBeg  Begin col index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount, sint32 SrcRowBeg, sint32 SrcColBeg, sint32 DstRowBeg, sint32 DstColBeg>
  void setBlock(const TriangularMatrix& block);

  /// \brief Multiplication with generic matrix: Tria * Matrix
  /// \tparam Cols_
  /// \tparam IsRowMajor_  
  /// \param[in] mat
  /// \return Matrix<ValueType_, Size_, Cols_, IsRowMajor_>
  template <sint32 Cols_, bool IsRowMajor_>
  auto operator*(const Matrix<ValueType_, Size_, Cols_, IsRowMajor_>& mat) const -> Matrix<ValueType_, Size_, Cols_, IsRowMajor_>;

  /// \brief Multiplication with triangular matrix: Tria * Matrix
  /// \param[in] mat  A triangular matrix
  /// \return TriangularMatrix<FloatType, Size, isLower>
  auto operator*(const TriangularMatrix& mat) const -> TriangularMatrix;

  /// \brief Multiplication with triangular matrix: Tria * Matrix
  /// \param[in] mat  A triangular matrix
  /// \return SquareMatrix<FloatType, Size>
  auto operator*(const TriangularMatrix<ValueType_, Size_, !IsLower_>& mat) const -> SquareMatrix;

  /// \brief Multiplication with diagonal matrix: Tria * Matrix
  /// \param[in] diag  A diagonal matrix
  /// \return TriangularMatrix<FloatType, Size, isLower>
  auto operator*(const DiagonalMatrix<ValueType_, Size_>& diag) const -> TriangularMatrix;

  /// \brief Multiplication with scalar: Tria * scalar
  /// \param[in] scalar  A scalar value
  /// \return TriangularMatrix<FloatType, Size, isLower>
  auto operator*(const ValueType_ scalar) const -> TriangularMatrix;

  /// \brief Inplace Multiplication with scalar: Tria * scalar
  /// \param[in] scalar  A scalar value
  /// \return TriangularMatrix<FloatType, Size, isLower>&
  auto operator*=(const ValueType_ scalar) -> TriangularMatrix&;

  /// \brief Element read-only access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType  scalar triangular value
  auto operator()(sint32 row, sint32 col) const -> tl::expected<ValueType_, Errors>;

  /// \brief Element access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType&  Reference to the scalar triangular value
  auto operator()(sint32 row, sint32 col) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>;

  /// \brief Calculate the transposed matrix
  /// \return TriangularMatrix<FloatType, Size, !isLower>
  auto transpose() const -> TriangularMatrix<ValueType_, Size_, !IsLower_>;

  /// \brief Solver for A*x=b based on Cholesky decomposition of A
  /// \tparam Cols_  Number of columns in the rhs variable b
  /// \tparam IsRowMajor_
  /// \param[in] b  Matrix describing the rhs of the equation A*x=b
  /// \return Matrix<FloatType, Size, Cols> describing x
  template <sint32 Cols_, bool IsRowMajor_>
  auto solve(const Matrix<ValueType_, Size_, Cols_, IsRowMajor_>& b) const -> Matrix<ValueType_, Size_, Cols_, IsRowMajor_>;

  /// \brief Calculates the inverse of the underlying matrix
  /// \return TriangularMatrix
  auto inverse() const -> TriangularMatrix;
  // TODO(matthias): UnitUpper inplace inverse, Grewal Table 6.7 p.235

  /// \brief Checks for Unit Upper condition
  /// \return true
  [[nodiscard]] auto isUnitUpperTriangular() const -> bool;

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Element read-only access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType  scalar triangular value
  auto at_unsafe(sint32 row, sint32 col) const -> ValueType_;

  /// \brief Element access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType&  Reference to the scalar triangular value
  auto at_unsafe(sint32 row, sint32 col) -> ValueType_&;
  // <---

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround to keep following idententation
  // clang-format on

  /// \brief hide inherited transpose function
  using SquareMatrix::transpose;

  /// \brief hide inherited operator() to prevent accessing off-triangular elements
  using SquareMatrix::operator();
};

} // namespace math
} // namespace tracking

#endif // BC7FD90F_FBB7_481C_89C4_89BEE41309C5
