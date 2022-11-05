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
/// \tparam FloatType
/// \tparam Rows
/// \tparam Cols
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixColumnView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a new Matrix Column View object
  /// \param[in] matrix    the viewed matrix
  /// \param[in] col       the selected column of matrix
  /// \param[in] rowBegin  optional begin row, default=0
  /// \param[in] rowEnd    optional end row, default=Rows-1
  explicit MatrixColumnView(const Matrix<FloatType, Rows, Cols>& matrix,
                            const sint32                         col,
                            const sint32                         rowBegin = 0,
                            const sint32                         rowEnd = Rows-1);

  /// \brief Read access to specific index of the column view
  /// \param[in] idx  index in range [0, rowCount]
  /// \return FloatType
  auto operator[](const sint32 idx) const -> FloatType;

  /// \brief Dot product between the viewed column and another column vector
  /// \tparam Rows2
  /// \param[in] other  A vector
  /// \return FloatType
  template <sint32 Rows2>
  auto operator*(const Vector<FloatType, Rows2>& other) const -> FloatType;

  /// \brief Dot product between the viewed column and another viewed column
  /// \tparam Rows2
  /// \tparam Cols2
  /// \param[in] other  A vector
  /// \return FloatType
  template <sint32 Rows2, sint32 Cols2>
  auto operator*(const MatrixColumnView<FloatType, Rows2, Cols2>& other) const -> FloatType;

  /// \brief Get the number of rows in the column view
  /// \return sint32
  [[nodiscard]] auto getRowCount() const -> sint32 { return _rowCount; }

private:
  const Matrix<FloatType, Rows, Cols>& _matrix;
  const sint32                         _col;
  const sint32                         _rowBegin;
  const sint32                         _rowCount;
};

/// \brief Matrix multiplication from left: Mat * ColView = Vector
/// \tparam FloatType 
/// \tparam Rows 
/// \tparam Cols 
/// \tparam Rows2 
/// \tparam Cols2 
/// \param[in] mat       A matrix
/// \param[in] colView   A matrix column view
/// \return Vector<FloatType, Rows> 
template <typename FloatType, sint32 Rows, sint32 Cols, sint32 Rows2, sint32 Cols2>
auto operator*(const Matrix<FloatType, Rows, Cols>& mat, const MatrixColumnView<FloatType, Rows2, Cols2>& colView)
    -> Vector<FloatType, Rows>;

} // namespace math
} // namespace tracking

#endif // ACBE61FE_A7AF_40EC_A62C_B94049A80274
