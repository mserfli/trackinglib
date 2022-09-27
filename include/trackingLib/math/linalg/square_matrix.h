#ifndef BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
#define BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08

#include "math/linalg/matrix.h"
#include <limits>

namespace tracking
{
namespace math
{

// forward declaration to prevent cyclic includes
template <typename FloatType, sint32 Size, bool isLower>
class TriangularMatrix;

// forward declaration to prevent cyclic includes
template <typename FloatType, sint32 Size>
class DiagonalMatrix;

// TODO(matthias): add interface contract
template <typename FloatType, sint32 Size>
class SquareMatrix: public Matrix<FloatType, Size, Size>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using Matrix<FloatType, Size, Size>::Matrix;

  /// \brief Construct a new Square Matrix< Float Type,  Size> object
  /// \param[in] other A base class object
  SquareMatrix<FloatType, Size>(const Matrix<FloatType, Size, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Set internal matrix to the Identity matrix
  void setIdentity();

  /// \brief Construct an Identity matrix
  /// \return SquareMatrix<FloatType, Size>  Resulting identity matrix
  static auto Identity() -> SquareMatrix<FloatType, Size>;

  /// \brief Decompose internal matrix into L*L' using standard Cholesky factorization
  /// \param[out] L  Calculated lower triangular matrix
  /// \return true   if calculation was successful
  // TODO(matthias): use std::expected or sth similar as result type
  auto decomposeLLT(TriangularMatrix<FloatType, Size, true>& L) const -> bool;

  /// \brief Decompose internal matrix into L*D*L' using rational Cholesky factorization
  /// \param[out] L  Calculated lower triangular matrix
  /// \param[out] D  Calculated diagonal matrix
  /// \return true   if calculation was successful
  // TODO(matthias): use std::expected or sth similar as result type
  auto decomposeLDLT(TriangularMatrix<FloatType, Size, true>& L, DiagonalMatrix<FloatType, Size>& D) const -> bool;

  /// \brief Decompose internal matrix into U*D*U' using rational Cholesky factorization
  /// \param[out] U  Calculated upper triangular matrix
  /// \param[out] D  Calculated diagonal matrix
  /// \return true   if calculation was successful
  // TODO(matthias): use std::expected or sth similar as result type
  auto decomposeUDUT(TriangularMatrix<FloatType, Size, false>& U, DiagonalMatrix<FloatType, Size>& D) const -> bool;
};

template <typename FloatType, sint32 Size>
SquareMatrix<FloatType, Size>::SquareMatrix(const Matrix<FloatType, Size, Size>& other)
    : Matrix<FloatType, Size, Size>{other}
{
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::decomposeLLT(TriangularMatrix<FloatType, Size, true>& L) const -> bool
{
  const Eigen::LLT<Eigen::Matrix<FloatType, Size, Size>> llt(this->_data);

  bool isOk = (llt.info() == Eigen::Success);
  if (isOk)
  {
    L.setZero();
    const auto& internalL = llt.matrixL();
    // copy lower triangular elements of internal matrix
    for (sint32 col = 0; col < Size; ++col)
    {
      L(col, col) = internalL(col, col);
      for (sint32 row = col + 1; row < Size; ++row)
      {
        L(row, col) = internalL(row, col);
      }
    }
  }
  return isOk;
}

template <typename FloatType, sint32 Size>
inline void SquareMatrix<FloatType, Size>::setIdentity()
{
  this->_data.setIdentity();
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::Identity() -> SquareMatrix<FloatType, Size>
{
  SquareMatrix<FloatType, Size> tmp;
  tmp.setIdentity();
  return tmp;
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::decomposeLDLT(TriangularMatrix<FloatType, Size, true>& L,
                                                         DiagonalMatrix<FloatType, Size>&         D) const -> bool
{
  const Eigen::SimplicialLDLT<Eigen::SparseMatrix<FloatType>, Eigen::Lower, Eigen::NaturalOrdering<int>> ldlt(
      this->_data.sparseView());

  bool isOk(ldlt.info() == Eigen::Success);
  if (isOk)
  {
    L.setIdentity();

    const auto  internalL = Eigen::SparseMatrix<FloatType>(ldlt.matrixL());
    const auto& internalD = ldlt.vectorD();
    // copy strict lower triangular and diagonal elements of internal matrix
    for (sint32 col = 0; col < Size; ++col)
    {
      D[col] = internalD[col];
      for (sint32 row = col + 1; row < Size; ++row)
      {
        L(row, col) = internalL.coeff(row, col);
      }
    }
  }
  return isOk;
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::decomposeUDUT(TriangularMatrix<FloatType, Size, false>& U,
                                                         DiagonalMatrix<FloatType, Size>&          D) const -> bool
{
  // Grewal & Andrews, Kalman Filtering Theory and Practice Using MATLAB, 4th
  // Edition, Wiley, 2014.
  //
  // Performs modified Cholesky decomposition of symmetric positive-definite
  // matrix P (input).

  auto P = static_cast<FloatType>(0.5) * (this->_data + this->_data.transpose());
  for (sint32 j = Size - 1; j >= 0; --j)
  {
    for (sint32 i = j; i >= 0; --i)
    {
      auto sigma = P(i, j);
      for (sint32 k = j + 1; k < Size; ++k)
      {
        sigma -= U(i, k) * D[k] * U(j, k);
      }
      if (i == j)
      {
        D[j] = std::max(sigma, std::numeric_limits<FloatType>::epsilon());
        U(j, j) = static_cast<FloatType>(1.0);
      }
      else
      {
        U(i, j) = sigma / D[j];
      }
    }
  }
  return true;
}

} // namespace math
} // namespace tracking
#endif // BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
