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

/// \brief A sub matrix view on a Matrix object providing arithmetic operations
/// \tparam FloatType
/// \tparam Rows
/// \tparam Cols
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixView TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a new Matrix View object
  /// \param[in] matrix    the viewed matrix
  /// \param[in] rowBegin
  /// \param[in] colBegin
  /// \param[in] rowEnd
  /// \param[in] colEnd
  explicit MatrixView(const Matrix<FloatType, Rows, Cols>& matrix,
                      const sint32                         rowBegin,
                      const sint32                         colBegin,
                      const sint32                         rowEnd,
                      const sint32                         colEnd);

  auto operator()(sint32 row, sint32 col) const -> FloatType;

  [[nodiscard]] auto getRowCount() const -> sint32 { return _rowCount; }
  [[nodiscard]] auto getColCount() const -> sint32 { return _colCount; }

  auto operator+(const Matrix<FloatType, Rows, Cols>& other) const -> Matrix<FloatType, Rows, Cols>;
  auto operator-(const Matrix<FloatType, Rows, Cols>& other) const -> Matrix<FloatType, Rows, Cols>;
  template <sint32 Cols2>
  auto operator*(const Matrix<FloatType, Cols, Cols2>& other) const -> Matrix<FloatType, Rows, Cols2>;

  auto operator+(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;
  auto operator-(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;
  auto operator*(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;
  auto operator/(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>;

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

template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
auto operator+(const MatrixView<FloatType, RowsA, ColsA>& a,
               const MatrixView<FloatType, RowsB, ColsB>& b) -> Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)>;

template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
auto operator-(const MatrixView<FloatType, RowsA, ColsA>& a,
               const MatrixView<FloatType, RowsB, ColsB>& b) -> Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)>;

template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
auto operator*(const MatrixView<FloatType, RowsA, ColsA>& a,
               const MatrixView<FloatType, RowsB, ColsB>& b) -> Matrix<FloatType, RowsA, ColsB>;


} // namespace math
} // namespace tracking

#endif // EE3A9F55_5D25_4810_82B7_406BD09800A1
