#ifndef EE3A9F55_5D25_4810_82B7_406BD09800A1
#define EE3A9F55_5D25_4810_82B7_406BD09800A1

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixColumnView;

template <typename FloatType, sint32 Rows, sint32 Cols, bool ExtMem = true>
class MatrixView TEST_REMOVE_FINAL
{
};


/// \brief A sub matrix view on a Matrix object providing arithmetic operations
/// \tparam FloatType
/// \tparam Rows
/// \tparam Cols
/// \tparam ExtMem  optionally use internal matrix object if false
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixView<FloatType, Rows, Cols, true> TEST_REMOVE_FINAL
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


template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixView<FloatType, Rows, Cols, false> TEST_REMOVE_FINAL
{
public:
  /// \brief Construct a new Matrix View object with internally stored matrix
  /// \param[in] rowBegin
  /// \param[in] colBegin
  /// \param[in] rowEnd
  /// \param[in] colEnd
  explicit MatrixView(const sint32 rowBegin, const sint32 colBegin, const sint32 rowEnd, const sint32 colEnd)
      : _view{_matrix, rowBegin, colBegin, rowEnd, colEnd}
  {
  }

  inline auto operator()(sint32 row, sint32 col) const -> FloatType { return _view(row, col); }
  inline auto operator()(sint32 row, sint32 col) -> FloatType { return _matrix(row, col); }

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on
  Matrix<FloatType, Rows, Cols>                  _matrix{};
  const MatrixView<FloatType, Rows, Cols, true>& _view;
};

template <typename FloatType, sint32 RowsA, sint32 ColsA, bool ExtMemA, sint32 RowsB, sint32 ColsB, bool ExtMemB>
auto operator+(const MatrixView<FloatType, RowsA, ColsA, ExtMemA>& a, const MatrixView<FloatType, RowsB, ColsB, ExtMemB>& b)
    -> MatrixView<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB), false>;

template <typename FloatType, sint32 RowsA, sint32 ColsA, bool ExtMemA, sint32 RowsB, sint32 ColsB, bool ExtMemB>
auto operator-(const MatrixView<FloatType, RowsA, ColsA, ExtMemA>& a, const MatrixView<FloatType, RowsB, ColsB, ExtMemB>& b)
    -> MatrixView<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB), false>;

template <typename FloatType, sint32 RowsA, sint32 ColsA, bool ExtMemA, sint32 RowsB, sint32 ColsB, bool ExtMemB>
auto operator*(const MatrixView<FloatType, RowsA, ColsA, ExtMemA>& a, const MatrixView<FloatType, RowsB, ColsB, ExtMemB>& b)
-> MatrixView<FloatType, RowsA, ColsB, false>;


} // namespace math
} // namespace tracking

#endif // EE3A9F55_5D25_4810_82B7_406BD09800A1
