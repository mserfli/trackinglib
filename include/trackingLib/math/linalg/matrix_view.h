#ifndef EE3A9F55_5D25_4810_82B7_406BD09800A1
#define EE3A9F55_5D25_4810_82B7_406BD09800A1

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class MatrixColumnView TEST_REMOVE_FINAL;

/// \brief Non-owning submatrix view providing arithmetic operations on Matrix objects
///
/// MatrixView provides a lightweight, zero-copy interface to access and manipulate
/// rectangular subregions of a Matrix. It supports arithmetic operations, matrix
/// multiplication, and element access without copying data.
///
/// \tparam FloatType The floating-point type for matrix elements (e.g., float32, float64)
/// \tparam Rows The number of rows in the viewed matrix
/// \tparam Cols The number of columns in the viewed matrix
///
/// \note This is a non-owning view - the lifetime of the MatrixView must not exceed
///       the lifetime of the underlying Matrix object.
/// \note All operations are performed in-place on the viewed subregion.
/// \note Performance: Zero-copy operations, minimal overhead compared to direct matrix access.
///
/// \see Matrix for the underlying matrix implementation
/// \see MatrixRowView for single-row views
/// \see MatrixColumnView for single-column views
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a submatrix view on an existing Matrix
  ///
  /// Creates a non-owning view of a rectangular subregion of the given matrix.
  /// The view covers rows from rowBegin to rowEnd-1 and columns from colBegin to colEnd-1.
  ///
  /// \param[in] matrix The matrix to view (must outlive the view)
  /// \param[in] rowBegin Starting row index (inclusive, 0-based)
  /// \param[in] colBegin Starting column index (inclusive, 0-based)
  /// \param[in] rowEnd Ending row index (exclusive, must be > rowBegin)
  /// \param[in] colEnd Ending column index (exclusive, must be > colBegin)
  ///
  /// \note The view does not own the data - ensure the matrix lifetime exceeds the view's
  /// \note Bounds are not checked at runtime - invalid indices may cause undefined behavior
  /// \note For full matrix views, use rowBegin=0, colBegin=0, rowEnd=Rows, colEnd=Cols
  explicit MatrixView(const Matrix<FloatType, Rows, Cols>& matrix,
                      const sint32                         rowBegin,
                      const sint32                         colBegin,
                      const sint32                         rowEnd,
                      const sint32                         colEnd);

  /// \brief Access element at specified position within the view
  ///
  /// \param[in] row Row index relative to the view (0 to getRowCount()-1)
  /// \param[in] col Column index relative to the view (0 to getColCount()-1)
  /// \return The element value
  ///
  /// \note No bounds checking - invalid indices cause undefined behavior
  [[nodiscard]] auto operator()(sint32 row, sint32 col) const -> FloatType;

  /// \brief Get the number of rows in this view
  /// \return Number of rows
  [[nodiscard]] auto getRowCount() const -> sint32 { return _rowCount; }
  /// \brief Get the number of columns in this view
  /// \return Number of columns
  [[nodiscard]] auto getColCount() const -> sint32 { return _colCount; }

  /// \brief Add this view to a full matrix
  ///
  /// Performs element-wise addition of the view's elements to the corresponding
  /// positions in the other matrix. The result is a full matrix.
  ///
  /// \param[in] other The matrix to add to
  /// \return New matrix containing the result
  [[nodiscard]] auto operator+(const Matrix<FloatType, Rows, Cols>& other) const -> Matrix<FloatType, Rows, Cols>;
  /// \brief Subtract a full matrix from this view
  ///
  /// Performs element-wise subtraction: view - other
  ///
  /// \param[in] other The matrix to subtract
  /// \return New matrix containing the result
  [[nodiscard]] auto operator-(const Matrix<FloatType, Rows, Cols>& other) const -> Matrix<FloatType, Rows, Cols>;
  /// \brief Multiply this view by another matrix
  ///
  /// Performs matrix multiplication: view * other
  ///
  /// \tparam Cols2 Number of columns in the other matrix
  /// \param[in] other Matrix to multiply with
  /// \return Result matrix of size Rows x Cols2
  template <sint32 Cols2>
  [[nodiscard]] auto operator*(const Matrix<FloatType, Cols, Cols2>& other) const -> Matrix<FloatType, Rows, Cols2>;

  /// \brief Add a scalar to all elements of this view
  ///
  /// \param[in] other Scalar value to add
  /// \return New matrix with scalar added to all elements
  [[nodiscard]] auto operator+(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;
  /// \brief Subtract a scalar from all elements of this view
  ///
  /// \param[in] other Scalar value to subtract
  /// \return New matrix with scalar subtracted from all elements
  [[nodiscard]] auto operator-(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;
  /// \brief Multiply all elements of this view by a scalar
  ///
  /// \param[in] other Scalar value to multiply by
  /// \return New matrix with all elements multiplied by the scalar
  [[nodiscard]] auto operator*(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;
  /// \brief Divide all elements of this view by a scalar
  ///
  /// \param[in] other Scalar value to divide by
  /// \return New matrix with all elements divided by the scalar
  [[nodiscard]] auto operator/(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on
  const Matrix<FloatType, Rows, Cols>& _matrix;
  const sint32                         _rowBegin;
  const sint32                         _colBegin;
  const sint32                         _rowCount;
  const sint32                         _colCount;
};

/// \brief Add two matrix views element-wise
///
/// Performs element-wise addition on the overlapping region.
/// The result dimensions are the minimum of the input dimensions.
///
/// \tparam RowsA Rows of first view
/// \tparam ColsA Cols of first view
/// \tparam RowsB Rows of second view
/// \tparam ColsB Cols of second view
/// \param[in] a First view
/// \param[in] b Second view
/// \return Matrix containing the element-wise sum
template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
[[nodiscard]] auto operator+(const MatrixView<FloatType, RowsA, ColsA>& a, const MatrixView<FloatType, RowsB, ColsB>& b)
    -> Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)>;

/// \brief Subtract two matrix views element-wise
///
/// Performs element-wise subtraction: a - b
///
/// \tparam RowsA Rows of first view
/// \tparam ColsA Cols of first view
/// \tparam RowsB Rows of second view
/// \tparam ColsB Cols of second view
/// \param[in] a First view
/// \param[in] b Second view
/// \return Matrix containing the element-wise difference
template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
[[nodiscard]] auto operator-(const MatrixView<FloatType, RowsA, ColsA>& a, const MatrixView<FloatType, RowsB, ColsB>& b)
    -> Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)>;

/// \brief Multiply two matrix views
///
/// Performs matrix multiplication: a * b
/// Requires that ColsA == RowsB
///
/// \tparam RowsA Rows of first view
/// \tparam ColsA Cols of first view
/// \tparam RowsB Rows of second view
/// \tparam ColsB Cols of second view
/// \param[in] a First view
/// \param[in] b Second view
/// \return Matrix containing the product
template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
[[nodiscard]] auto operator*(const MatrixView<FloatType, RowsA, ColsA>& a,
                             const MatrixView<FloatType, RowsB, ColsB>& b) -> Matrix<FloatType, RowsA, ColsB>;


} // namespace math
} // namespace tracking

#endif // EE3A9F55_5D25_4810_82B7_406BD09800A1
