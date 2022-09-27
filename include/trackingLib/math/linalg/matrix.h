#ifndef FDEAAACC_9EF1_4C87_94DC_2FA494822664
#define FDEAAACC_9EF1_4C87_94DC_2FA494822664

#include "base/atomic_types.h"
#include "math/linalg/contracts/matrix_intf.h"
#include <cmath>
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

template <typename FloatType, sint32 Rows, sint32 Cols>
class Matrix : public contract::MatrixIntf<Matrix<FloatType, Rows, Cols>>
{
public:
  using self = Matrix<FloatType, Rows, Cols>;

  Matrix() = default;
  Matrix(const Matrix<FloatType, Rows, Cols>&) = default;
  Matrix(Matrix<FloatType, Rows, Cols>&&) noexcept = default;
  auto operator=(const Matrix<FloatType, Rows, Cols>&) -> Matrix<FloatType, Rows, Cols>& = default;
  auto operator=(Matrix<FloatType, Rows, Cols>&&) noexcept -> Matrix<FloatType, Rows, Cols>& = default;

  /// \brief Construct a new Matrix object given initializer list
  /// \param[in] list  An initializer list describing list of matrix rows
  Matrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  auto operator()(sint32 row, sint32 col) const -> FloatType;
  auto operator()(sint32 row, sint32 col) -> FloatType&;

  auto operator+=(const self& other) -> self&;
  auto operator-=(const self& other) -> self&;

  template <sint32 Cols2>
  auto operator*=(const Matrix<FloatType, Cols, Cols2>& other) -> Matrix<FloatType, Rows, Cols2>;

  auto operator*=(FloatType scalar) -> self&;
  auto operator/=(FloatType scalar) -> self&;

  void        setZero();
  static auto Zero() -> self;

  auto transpose() const -> Matrix<FloatType, Cols, Rows>;

  void print() const;

protected:
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
inline auto Matrix<FloatType, Rows, Cols>::operator()(sint32 row, sint32 col) const -> FloatType
{
  assert((row>=0 && row<Rows) && "bad row index");
  assert((col>=0 && col<Cols) && "bad col index");
  return _data(row, col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator()(sint32 row, sint32 col) -> FloatType&
{
  assert((row>=0 && row<Rows) && "bad row index");
  assert((col>=0 && col<Cols) && "bad col index");
  return _data(row, col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator+=(const self& other) -> self&
{
  _data += other._data;
  return *this;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator-=(const self& other) -> self&
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
inline auto Matrix<FloatType, Rows, Cols>::operator*=(FloatType scalar) -> self&
{
  _data *= scalar;
  return *this;
}
template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto Matrix<FloatType, Rows, Cols>::operator/=(FloatType scalar) -> self&
{
  assert(std::abs(scalar)>static_cast<FloatType>(0.0));
  _data /= scalar;
  return *this;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline void Matrix<FloatType, Rows, Cols>::setZero()
{
  _data.setZero();
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto Matrix<FloatType, Rows, Cols>::Zero() -> self
{
  self tmp;
  tmp.setZero();
  return tmp;
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
  assert(std::abs(scalar)>static_cast<FloatType>(0.0));
  Matrix<FloatType, Rows, Cols> temp(m);
  return (temp /= scalar);
}

} // namespace math
} // namespace tracking

#endif // FDEAAACC_9EF1_4C87_94DC_2FA494822664
