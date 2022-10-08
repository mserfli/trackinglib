#ifndef BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
#define BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08

#include "base/first_include.h"
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
  SquareMatrix(const Matrix<FloatType, Size, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Set internal matrix to the Identity matrix
  void setIdentity();

  /// \brief Construct an Identity matrix
  /// \return SquareMatrix  Resulting identity matrix
  static auto Identity() -> SquareMatrix;

  auto qrSolve(SquareMatrix<FloatType, Size>& x, const SquareMatrix<FloatType, Size>& b) const -> bool;

  /// \brief Decompose internal matrix into L*L' using standard Cholesky factorization
  /// \param[out] L  Calculated lower triangular matrix
  /// \return true   if calculation was successful
  /// \precondition internal matrix is symmetric and positive definite
  // TODO(matthias): use std::expected or sth similar as result type
  auto decomposeLLT(TriangularMatrix<FloatType, Size, true>& L) const -> bool;

  /// \brief Decompose internal matrix into L*D*L' using rational Cholesky factorization
  /// \param[out] L  Calculated lower triangular matrix
  /// \param[out] D  Calculated diagonal matrix
  /// \return true   if calculation was successful
  /// \precondition internal matrix is symmetric and positive semi definite
  // TODO(matthias): use std::expected or sth similar as result type
  auto decomposeLDLT(TriangularMatrix<FloatType, Size, true>& L, DiagonalMatrix<FloatType, Size>& D) const -> bool;

  /// \brief Decompose internal matrix into U*D*U' using rational Cholesky factorization
  /// \param[out] U  Calculated upper triangular matrix
  /// \param[out] D  Calculated diagonal matrix
  /// \return true   if calculation was successful
  /// \precondition internal matrix is symmetric and positive semi definite
  // TODO(matthias): use std::expected or sth similar as result type
  auto decomposeUDUT(TriangularMatrix<FloatType, Size, false>& U, DiagonalMatrix<FloatType, Size>& D) const -> bool;

  /// \brief Calculates the inverse based on Cholesky factorization
  /// \return SquareMatrix
  auto inverse() const -> SquareMatrix;

  // TODO(matthias): add function to check symmetry required for all decompose functions
};

} // namespace math
} // namespace tracking
#endif // BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
