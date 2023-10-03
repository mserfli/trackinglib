#ifndef A45AF6CD_F19E_48A5_92B4_D7A86498C01A
#define A45AF6CD_F19E_48A5_92B4_D7A86498C01A

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class MatrixColumnView;

/// \brief A row view on a Matrix object providing arithmetic operations
/// \tparam ValueType_
/// \tparam Rows_
/// \tparam Cols_
/// \tparam IsRowMajor_
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class MatrixRowView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a new Matrix Row View object
  /// \param[in] matrix    the viewed matrix
  /// \param[in] row       the selected row of matrix
  /// \param[in] colBegin  optional begin col, default=0
  /// \param[in] colEnd  optional number of cols, default=Cols_-1
  explicit MatrixRowView(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& matrix,
                         const sint32                                         row,
                         const sint32                                         colBegin = 0,
                         const sint32                                         colEnd   = Cols_ - 1);

  /// \brief Read access to specific index of the row view
  /// \param[in] idx  index in range [0, colCount]
  /// \return ValueType_
  auto at_unsafe(const sint32 idx) const -> ValueType_;

  /// \brief Multiplication with other matrix
  /// \tparam Rows2_
  /// \tparam Cols2_
  /// \tparam IsRowMajor2_
  /// \param[in] other  A matrix
  /// \return Matrix<ValueType_, 1, Cols2_, IsRowMajor2_>
  template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
  auto operator*(const Matrix<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const
      -> Matrix<ValueType_, 1, Cols2_, IsRowMajor2_>;

  /// \brief Dot product with other matrix column view
  /// \tparam Rows2_
  /// \tparam Cols2_
  /// \tparam IsRowMajor2_
  /// \param[in] other  A matrix column view
  /// \return ValueType_
  template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
  auto operator*(const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const -> ValueType_;

  /// \brief Get the number of cols in the row view
  /// \return sint32
  [[nodiscard]] auto getColCount() const -> sint32 { return _colCount; }

  /// \brief Print the matrix to stdout
  void print() const;

private:
  const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& _matrix;
  const sint32                                         _row;
  const sint32                                         _colBegin;
  const sint32                                         _colCount;
};

} // namespace math
} // namespace tracking

#endif // A45AF6CD_F19E_48A5_92B4_D7A86498C01A
