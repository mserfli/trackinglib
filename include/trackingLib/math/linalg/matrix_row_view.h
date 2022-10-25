#ifndef A45AF6CD_F19E_48A5_92B4_D7A86498C01A
#define A45AF6CD_F19E_48A5_92B4_D7A86498C01A

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixColumnView;

/// \brief A row view on a Matrix object providing arithmetic operations
/// \tparam FloatType
/// \tparam Rows
/// \tparam Cols
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixRowView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a new Matrix Row View object
  /// \param[in] matrix    the viewed matrix
  /// \param[in] row       the selected row of matrix
  /// \param[in] colBegin  optional begin col, default=0
  /// \param[in] colEnd  optional number of cols, default=Cols
  explicit MatrixRowView(const Matrix<FloatType, Rows, Cols>& matrix,
                         const sint32                         row,
                         const sint32                         colBegin = 0,
                         const sint32                         colEnd = Cols-1);

  /// \brief Read access to specific index of the row view
  /// \param[in] idx  index in range [0, colCount]
  /// \return FloatType
  auto operator[](const sint32 idx) const -> FloatType;

  /// \brief Multiplication with other matrix
  /// \tparam Rows2
  /// \tparam Cols2
  /// \param[in] other  A matrix
  /// \return Matrix<FloatType, 1, Cols2>
  template <sint32 Rows2, sint32 Cols2>
  auto operator*(const Matrix<FloatType, Rows2, Cols2>& other) const -> Matrix<FloatType, 1, Cols2>;

  /// \brief Dot product with other matrix column view
  /// \tparam Rows2
  /// \tparam Cols2
  /// \param[in] other  A matrix column view
  /// \return FloatType
  template <sint32 Rows2, sint32 Cols2>
  auto operator*(const MatrixColumnView<FloatType, Rows2, Cols2>& other) const -> FloatType;

  /// \brief Get the number of cols in the row view
  /// \return sint32
  auto getColCount() const -> sint32 { return _colCount; }

private:
  const Matrix<FloatType, Rows, Cols>& _matrix;
  const sint32                         _row;
  const sint32                         _colBegin;
  const sint32                         _colCount;
};

/// \brief Dot product between vector and a matrix row view: Vector * RowView = Scalar
/// \tparam FloatType
/// \tparam Rows
/// \tparam Rows2
/// \tparam Cols2
/// \param[in] vec  A vector
/// \param[in] rowView  A matrix row view
/// \return FloatType
template <typename FloatType, sint32 Rows, sint32 Rows2, sint32 Cols2>
auto operator*(const Vector<FloatType, Rows>& vec, const MatrixRowView<FloatType, Rows2, Cols2>& rowView) -> FloatType;

} // namespace math
} // namespace tracking

#endif // A45AF6CD_F19E_48A5_92B4_D7A86498C01A
