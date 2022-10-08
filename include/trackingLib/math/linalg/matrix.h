#ifndef FDEAAACC_9EF1_4C87_94DC_2FA494822664
#define FDEAAACC_9EF1_4C87_94DC_2FA494822664

#include "base/first_include.h"
#include "math/linalg/contracts/matrix_intf.h"
#include <cmath>
#include <initializer_list>
#include <iostream>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseCholesky>
#pragma clang diagnostic pop


namespace tracking
{
namespace math
{

// forward declaration to prevent cyclic includes
template <typename FloatType, sint32 Size, bool isLower>
class TriangularMatrix;

// TODO(matthias): add doxygen
// TODO(matthias): add support for external memory
// TODO(matthias): add template params to support coord transformations (inFrame e.g. EGO_k_1, outFrame, power -> 2 for covs)
template <typename FloatType, sint32 Rows, sint32 Cols>
class Matrix: public contract::MatrixIntf<Matrix<FloatType, Rows, Cols>, Matrix>
{
public:
  using transpose_type       = Matrix<FloatType, Cols, Rows>;
  using value_type           = FloatType;
  static constexpr auto rows = Rows;
  static constexpr auto cols = Cols;

  Matrix()                  = default;
  Matrix(const Matrix&)     = default;
  Matrix(Matrix&&) noexcept = default;
  auto operator=(const Matrix&) -> Matrix& = default;
  auto operator=(Matrix&&) noexcept -> Matrix& = default;

  /// \brief Construct a new Matrix object given initializer list
  /// \param[in] list  An initializer list describing list of matrix rows
  Matrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  explicit Matrix(const std::array<FloatType, static_cast<size_t>(Rows* Cols)>& arr);

  auto operator()(sint32 row, sint32 col) const -> FloatType;
  auto operator()(sint32 row, sint32 col) -> FloatType&;

  auto operator+=(const Matrix& other) -> Matrix&;
  auto operator-=(const Matrix& other) -> Matrix&;

  template <sint32 Cols2>
  auto operator*=(const Matrix<FloatType, Cols, Cols2>& other) -> Matrix<FloatType, Rows, Cols2>;

  auto operator*=(FloatType scalar) -> Matrix&;
  auto operator/=(FloatType scalar) -> Matrix&;

  void        setZeros();
  static auto Zeros() -> Matrix;

  void        setOnes();
  static auto Ones() -> Matrix;

  /// \brief Set a block matrix at given position
  /// \tparam SrcRowSize   Rows of the source block
  /// \tparam SrcColSize   Cols of the source block
  /// \tparam SrcRowCount  Number of rows to copy from source
  /// \tparam SrcColCount  Number of cols to copy from source
  /// \tparam SrcRowBeg    Begin row index in source
  /// \tparam SrcColBeg    Begin col index in source
  /// \tparam DstRowBeg    Begin row index in dest
  /// \tparam DstColBeg    Begin col index in dest
  /// \param[in] block     Source block matrix to copy from
  template <sint32 SrcRowSize,
            sint32 SrcColSize,
            sint32 SrcRowCount,
            sint32 SrcColCount,
            sint32 SrcRowBeg,
            sint32 SrcColBeg,
            sint32 DstRowBeg,
            sint32 DstColBeg>
  void setBlock(const Matrix<FloatType, SrcRowSize, SrcColSize>& block);

  auto transpose() const -> transpose_type;

  void print() const;

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on
  Eigen::Matrix<FloatType, Rows, Cols> _data{}; // TODO(matthias): make this a unique_ptr to profit from move ctor/assignment

  template <typename U, sint32 Rows_, sint32 Cols_>
  friend class Matrix; // needed to access member _data in operator*=

  template <typename U, sint32 Size, bool isLower>
  friend class TriangularMatrix; // needed to access member _data in solve()
};

//
// implementation of member functions/operators

template <typename FloatType, sint32 Rows, sint32 Cols>
Matrix<FloatType, Rows, Cols>::Matrix(const std::initializer_list<std::initializer_list<FloatType>>& list)
    : _data{list}
{
}

template <typename FloatType, sint32 Rows, sint32 Cols>
Matrix<FloatType, Rows, Cols>::Matrix(const std::array<FloatType, static_cast<size_t>(Rows* Cols)>& arr)
    : _data{Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(arr.data(), Rows, Cols)}
{
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator()(sint32 row, sint32 col) const -> FloatType
{
  assert((row >= 0 && row < Rows) && "bad row index");
  assert((col >= 0 && col < Cols) && "bad col index");
  return _data(row, col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator()(sint32 row, sint32 col) -> FloatType&
{
  assert((row >= 0 && row < Rows) && "bad row index");
  assert((col >= 0 && col < Cols) && "bad col index");
  return _data(row, col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator+=(const Matrix& other) -> Matrix&
{
  _data += other._data;
  return *this;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator-=(const Matrix& other) -> Matrix&
{
  _data -= other._data;
  return *this;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Cols2>
inline auto Matrix<FloatType, Rows, Cols>::operator*=(const Matrix<FloatType, Cols, Cols2>& other)
    -> Matrix<FloatType, Rows, Cols2>
{
  Matrix<FloatType, Rows, Cols2> temp;
  temp._data = (_data * other._data).eval();
  return temp;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator*=(FloatType scalar) -> Matrix&
{
  _data *= scalar;
  return *this;
}
template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator/=(FloatType scalar) -> Matrix&
{
  assert(std::abs(scalar) > static_cast<FloatType>(0.0));
  _data /= scalar;
  return *this;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline void Matrix<FloatType, Rows, Cols>::setZeros()
{
  _data.setZero();
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto Matrix<FloatType, Rows, Cols>::Zeros() -> Matrix
{
  Matrix tmp;
  tmp.setZeros();
  return tmp;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline void Matrix<FloatType, Rows, Cols>::setOnes()
{
  _data.setOnes();
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto Matrix<FloatType, Rows, Cols>::Ones() -> Matrix
{
  Matrix tmp;
  tmp.setOnes();
  return tmp;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 SrcRowSize,
          sint32 SrcColSize,
          sint32 SrcRowCount,
          sint32 SrcColCount,
          sint32 SrcRowBeg,
          sint32 SrcColBeg,
          sint32 DstRowBeg,
          sint32 DstColBeg>
void Matrix<FloatType, Rows, Cols>::setBlock(const Matrix<FloatType, SrcRowSize, SrcColSize>& block)
{
  static_assert((SrcRowCount > 1) && (SrcColCount > 1), "use scalar access operator for block copy size == 1");
  static_assert(SrcRowBeg + SrcRowCount <= SrcRowSize, "copy to many rows from src");
  static_assert(SrcColBeg + SrcColCount <= SrcColSize, "copy to many cols from src");

  static_assert(DstRowBeg + SrcRowCount <= Rows, "copy to many rows to dst");
  static_assert(DstColBeg + SrcColCount <= Cols, "copy to many cols to dst");

  for (sint32 row = 0; row < SrcRowCount; ++row)
  {
    for (sint32 col = 0; col < SrcColCount; ++col)
    {
      this->operator()(DstRowBeg + row, DstColBeg + col) = block(SrcRowBeg + row, SrcColBeg + col);
    }
  }
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::transpose() const -> Matrix<FloatType, Cols, Rows>
{
  Matrix<FloatType, Cols, Rows> temp;
  temp._data = _data.transpose();
  return temp;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline void Matrix<FloatType, Rows, Cols>::print() const
{
  std::cout << _data << "\n" << std::endl;
}


//
// non member class operators

template <typename FloatType, sint32 Rows, sint32 Cols>
auto operator+(const Matrix<FloatType, Rows, Cols>& m1, const Matrix<FloatType, Rows, Cols>& m2) -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> temp(m1);
  return (temp += m2);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto operator-(const Matrix<FloatType, Rows, Cols>& m1, const Matrix<FloatType, Rows, Cols>& m2) -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> temp(m1);
  return (temp -= m2);
}

template <typename FloatType, sint32 Rows1, sint32 Cols1, sint32 Cols2>
auto operator*(const Matrix<FloatType, Rows1, Cols1>& m1, const Matrix<FloatType, Cols1, Cols2>& m2)
    -> Matrix<FloatType, Rows1, Cols2>
{
  Matrix<FloatType, Rows1, Cols1> temp(m1);
  return (temp *= m2);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto operator*(const Matrix<FloatType, Rows, Cols>& m, FloatType scalar) -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> temp(m);
  return (temp *= scalar);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto operator*(FloatType scalar, const Matrix<FloatType, Rows, Cols>& m) -> Matrix<FloatType, Rows, Cols>
{
  return (m * scalar);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto operator/(const Matrix<FloatType, Rows, Cols>& m, FloatType scalar) -> Matrix<FloatType, Rows, Cols>
{
  assert(std::abs(scalar) > static_cast<FloatType>(0.0));
  Matrix<FloatType, Rows, Cols> temp(m);
  return (temp /= scalar);
}

} // namespace math
} // namespace tracking

#endif // FDEAAACC_9EF1_4C87_94DC_2FA494822664
