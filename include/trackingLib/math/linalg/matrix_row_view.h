#ifndef A45AF6CD_F19E_48A5_92B4_D7A86498C01A
#define A45AF6CD_F19E_48A5_92B4_D7A86498C01A

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/matrix.h" // IWYU pragma: keep
#include "math/linalg/vector.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class MatrixColumnView;

/// \brief Non-owning single-row view providing arithmetic operations on Matrix objects
///
/// MatrixRowView provides a lightweight, zero-copy interface to access and manipulate
/// a single row of a Matrix. It supports matrix multiplication and element access
/// without copying data.
///
/// \tparam ValueType_ The value type for matrix elements
/// \tparam Rows_ The number of rows in the source matrix
/// \tparam Cols_ The number of columns in the source matrix
/// \tparam IsRowMajor_ The storage layout of the source matrix (default: true)
///
/// \note This is a non-owning view - the lifetime of the MatrixRowView must not exceed
///       the lifetime of the underlying Matrix object.
/// \note All operations are performed in-place on the viewed row.
/// \note Performance: Zero-copy operations, minimal overhead compared to direct matrix access.
///
/// \see Matrix for the underlying matrix implementation
/// \see MatrixView for rectangular submatrix views
/// \see MatrixColumnView for single-column views
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class MatrixRowView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a single-row view on an existing Matrix
  ///
  /// Creates a non-owning view of a single row of the given matrix.
  /// The view covers columns from colBegin to colEnd-1 of the specified row.
  ///
  /// \param[in] matrix The matrix to view (must outlive the view)
  /// \param[in] row Row index to view (0-based)
  /// \param[in] colBegin Starting column index (0-based, default: 0)
  /// \param[in] colEnd Ending column index (0-based, default: Cols_-1)
  ///
  /// \note The view does not own the data - ensure the matrix lifetime exceeds the view's
  /// \note Bounds are not checked at runtime - invalid indices may cause undefined behavior
  /// \note For full row views, use colBegin=0, colEnd=Cols_-1
  explicit MatrixRowView(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& matrix,
                         const sint32                                         row,
                         const sint32                                         colBegin = 0,
                         const sint32                                         colEnd   = Cols_ - 1);

  /// \brief Access element at specified column index within the row view
  ///
  /// \param[in] idx Column index relative to the view (0 to getColCount()-1)
  /// \return The element value
  ///
  /// \note No bounds checking - invalid indices cause undefined behavior
  [[nodiscard]] auto at_unsafe(const sint32 idx) const -> ValueType_;

  /// \brief Multiply this row view by another matrix
  ///
  /// Performs matrix multiplication: row_view * other
  /// The result is a 1xCols2 matrix (effectively a row vector).
  ///
  /// \tparam Rows2_ Rows of the other matrix
  /// \tparam Cols2_ Cols of the other matrix
  /// \tparam IsRowMajor2_ Storage layout of the other matrix
  /// \param[in] other Matrix to multiply with
  /// \return Result matrix of size 1 x Cols2_
  template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
  [[nodiscard]] auto operator*(const Matrix<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const
      -> Matrix<ValueType_, 1, Cols2_, IsRowMajor2_>;

  /// \brief Compute dot product with a column view
  ///
  /// Performs dot product: row_view • column_view
  /// Requires that the view dimensions are compatible.
  ///
  /// \tparam Rows2_ Rows of the column view
  /// \tparam Cols2_ Cols of the column view
  /// \tparam IsRowMajor2_ Storage layout of the column view
  /// \param[in] other Column view to multiply with
  /// \return Scalar result of the dot product
  template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
  [[nodiscard]] auto operator*(const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const -> ValueType_;

  /// \brief Get the number of columns in this row view
  /// \return Number of columns
  [[nodiscard]] auto getColCount() const -> sint32 { return _colCount; }


private:
  const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& _matrix;
  const sint32                                         _row;
  const sint32                                         _colBegin;
  const sint32                                         _colCount;
};

} // namespace math
} // namespace tracking

#endif // A45AF6CD_F19E_48A5_92B4_D7A86498C01A
