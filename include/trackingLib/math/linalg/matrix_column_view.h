#ifndef ACBE61FE_A7AF_40EC_A62C_B94049A80274
#define ACBE61FE_A7AF_40EC_A62C_B94049A80274

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

/// \brief A column view on a Matrix object providing arithmetic operations
/// \tparam ValueType_
/// \tparam Rows_
/// \tparam Cols_
/// \tparam IsRowMajor_
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class MatrixColumnView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a new Matrix Column View object
  /// \param[in] matrix    the viewed matrix
  /// \param[in] col       the selected column of matrix
  /// \param[in] rowBegin  optional begin row, default=0
  /// \param[in] rowEnd    optional end row, default=Rows_-1
  explicit MatrixColumnView(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& matrix,
                            const sint32                                         col,
                            const sint32                                         rowBegin = 0,
                            const sint32                                         rowEnd   = Rows_ - 1);

  /// \brief Read access to specific index of the column view
  /// \param[in] idx  index in range [0, rowCount]
  /// \return ValueType_
  auto at_unsafe(const sint32 idx) const -> ValueType_;

  /// \brief Dot product between the viewed column and another column vector
  /// \tparam Rows2_
  /// \param[in] other  A vector
  /// \return ValueType_
  template <sint32 Rows2_>
  auto operator*(const Vector<ValueType_, Rows2_>& other) const -> ValueType_;

  /// \brief Dot product between the viewed column and another viewed column
  /// \tparam Rows2_
  /// \tparam Cols2_
  /// \tparam IsRowMajor2_
  /// \param[in] other  Another column view
  /// \return ValueType_
  template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
  auto operator*(const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const -> ValueType_;

  // TODO(matthias): add columnView * rowView resulting in a new Matrix

  /// \brief Get the number of rows in the column view
  /// \return sint32
  [[nodiscard]] auto getRowCount() const -> sint32 { return _rowCount; }

  /// \brief Print the matrix to stdout
  void print() const;

private:
  const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& _matrix;
  const sint32                                         _col;
  const sint32                                         _rowBegin;
  const sint32                                         _rowCount;
};

/// \brief Matrix multiplication from left: Mat * ColView = Vector
/// \tparam ValueType_
/// \tparam Rows_
/// \tparam Cols_
/// \tparam Rows2_
/// \tparam Cols2_
/// \param[in] mat       A matrix
/// \param[in] colView   A matrix column view
/// \return Vector<ValueType_, Rows_>
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_, sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
auto operator*(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>&              mat,
               const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& colView) -> Vector<ValueType_, Rows_>;

} // namespace math
} // namespace tracking

#endif // ACBE61FE_A7AF_40EC_A62C_B94049A80274
