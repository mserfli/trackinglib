#ifndef ACBE61FE_A7AF_40EC_A62C_B94049A80274
#define ACBE61FE_A7AF_40EC_A62C_B94049A80274

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/matrix.h" // IWYU pragma: keep
#include "math/linalg/vector.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

/// \brief Non-owning single-column view providing arithmetic operations on Matrix objects
///
/// MatrixColumnView provides a lightweight, zero-copy interface to access and manipulate
/// a single column of a Matrix. It supports dot product operations and element access
/// without copying data.
///
/// \tparam ValueType_ The atomic data type of internal elements
/// \tparam Rows_ The number of rows in the source matrix
/// \tparam Cols_ The number of columns in the source matrix
/// \tparam IsRowMajor_ The storage layout of the source matrix (default: true)
///
/// \note This is a non-owning view - the lifetime of the MatrixColumnView must not exceed
///       the lifetime of the underlying Matrix object.
/// \note All operations are performed in-place on the viewed column.
/// \note Performance: Zero-copy operations, minimal overhead compared to direct matrix access.
///
/// \see Matrix for the underlying matrix implementation
/// \see MatrixView for rectangular submatrix views
/// \see MatrixRowView for single-row views
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class MatrixColumnView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a single-column view on an existing Matrix
  ///
  /// Creates a non-owning view of a single column of the given matrix.
  /// The view covers rows from rowBegin to rowEnd-1 of the specified column.
  ///
  /// \param[in] matrix The matrix to view (must outlive the view)
  /// \param[in] col Column index to view (0-based)
  /// \param[in] rowBegin Starting row index (0-based, default: 0)
  /// \param[in] rowEnd Ending row index (0-based, default: Rows_-1)
  ///
  /// \note The view does not own the data - ensure the matrix lifetime exceeds the view's
  /// \note Bounds are not checked at runtime - invalid indices may cause undefined behavior
  /// \note For full column views, use rowBegin=0, rowEnd=Rows_-1
  explicit MatrixColumnView(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& matrix,
                            const sint32                                         col,
                            const sint32                                         rowBegin = 0,
                            const sint32                                         rowEnd   = Rows_ - 1);

  /// \brief Access element at specified row index within the column view
  ///
  /// \param[in] idx Row index relative to the view (0 to getRowCount()-1)
  /// \return The element value
  ///
  /// \note No bounds checking - invalid indices cause undefined behavior
  [[nodiscard]] auto at_unsafe(const sint32 idx) const -> ValueType_;

  /// \brief Compute dot product with a vector
  ///
  /// Performs dot product: column_view • vector
  /// Requires that the view and vector dimensions are compatible.
  ///
  /// \tparam Rows2_ Size of the vector
  /// \param[in] other Vector to multiply with
  /// \return Scalar result of the dot product
  template <sint32 Rows2_>
  [[nodiscard]] auto operator*(const Vector<ValueType_, Rows2_>& other) const -> ValueType_;

  /// \brief Compute dot product with another column view
  ///
  /// Performs dot product: column_view1 • column_view2
  /// Requires that the view dimensions are compatible.
  ///
  /// \tparam Rows2_ Rows of the other column view
  /// \tparam Cols2_ Cols of the other column view
  /// \tparam IsRowMajor2_ Storage layout of the other column view
  /// \param[in] other Column view to multiply with
  /// \return Scalar result of the dot product
  template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
  [[nodiscard]] auto operator*(const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const -> ValueType_;

  // TODO(matthias): add columnView * rowView resulting in a new Matrix

  /// \brief Get the number of rows in this column view
  /// \return Number of rows
  [[nodiscard]] auto getRowCount() const -> sint32 { return _rowCount; }


private:
  const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& _matrix;
  const sint32                                         _col;
  const sint32                                         _rowBegin;
  const sint32                                         _rowCount;
};

/// \brief Multiply a matrix by a column view
///
/// Performs matrix multiplication: mat * colView
/// The result is a vector where each element is the dot product of the corresponding
/// matrix row with the column view.
///
/// \tparam ValueType_ The atomic data type of internal elements
/// \tparam Rows_ Rows of the matrix
/// \tparam Cols_ Cols of the matrix
/// \tparam IsRowMajor_ Storage layout of the matrix
/// \tparam Rows2_ Rows of the column view
/// \tparam Cols2_ Cols of the column view
/// \tparam IsRowMajor2_ Storage layout of the column view
/// \param[in] mat Matrix to multiply
/// \param[in] colView Column view to multiply with
/// \return Vector result of size Rows_
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_, sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
[[nodiscard]] auto operator*(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>&              mat,
                             const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& colView)
    -> Vector<ValueType_, Rows_>;

} // namespace math
} // namespace tracking

#endif // ACBE61FE_A7AF_40EC_A62C_B94049A80274
