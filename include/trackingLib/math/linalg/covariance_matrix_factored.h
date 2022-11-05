#ifndef F9044FD7_A3A8_43F4_BDD6_F43011384722
#define F9044FD7_A3A8_43F4_BDD6_F43011384722

#include "base/first_include.h"
#include "base/atomic_types.h"
#include "math/linalg/contracts/covariance_matrix_intf.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.h"
#include "math/linalg/triangular_matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{
// TODO(matthias): add contract for apaT functions, fill, ...
template <typename FloatType, sint32 Size>
class CovarianceMatrixFactored: public contract::CovarianceMatrixIntf<CovarianceMatrixFactored<FloatType, Size>>
{
public:
  using value_type   = FloatType;
  using compose_type = CovarianceMatrixFull<FloatType, Size>;
  static constexpr auto dim = Size;

  // rule of 5 declarations
  CovarianceMatrixFactored()                                    = default;
  CovarianceMatrixFactored(const CovarianceMatrixFactored&)     = default;
  CovarianceMatrixFactored(CovarianceMatrixFactored&&) noexcept = default;
  auto operator=(const CovarianceMatrixFactored&) -> CovarianceMatrixFactored& = default;
  auto operator=(CovarianceMatrixFactored&&) noexcept -> CovarianceMatrixFactored& = default;
  virtual ~CovarianceMatrixFactored()                                              = default;

  explicit CovarianceMatrixFactored(const SquareMatrix<FloatType, Size>& other, const bool isInverse = false);
  explicit CovarianceMatrixFactored(const TriangularMatrix<FloatType, Size, false>& u,
                                    const DiagonalMatrix<FloatType, Size>&          d,
                                    const bool                                      isInverse = false);

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFactored
  static auto Identity() -> CovarianceMatrixFactored;

  /// \brief Set Identity covariance
  void setIdentity();

  /// \brief Access operator to the covariance value at (row, col)
  /// \param[in,out] row  The specified row
  /// \param[in,out] col  The specified column
  /// \return FloatType
  auto operator()(sint32 row, sint32 col) const -> FloatType;

  /// \brief Creates the composed covariance
  /// \return CovarianceMatrixFull<FloatType, Size>
  auto operator()() const -> CovarianceMatrixFull<FloatType, Size>;

  /// \brief Calculates the inverse
  /// \return CovarianceMatrixFactored
  auto inverse() const -> CovarianceMatrixFactored;

  /// \brief
  /// \return true
  /// \return false
  [[nodiscard]] auto isInverse() const -> bool;

  /// \brief Calculate A*P*A' inplace
  /// \param[in] A   A square matrix which is transforming P in same space
  void apaT(const SquareMatrix<FloatType, Size>& A);

  /// \brief Calculate A*P*A'
  /// \param[in] A   A square matrix which is transforming P in same space
  /// \return Calculation result as new covariance matrix
  auto apaT(const SquareMatrix<FloatType, Size>& A) const -> CovarianceMatrixFactored;

  /// \brief Calculate Phi*P*Phi' + G*Q*G', also known as Thornton update
  /// \tparam SizeQ
  /// \param[in] Phi  A square matrix which is transforming P in same space
  /// \param[in] G    A matrix to transform Q into matrix equally sized to Phi
  /// \param[in] Q    A diagonal matrix
  template <sint32 SizeQ>
  void thornton(const SquareMatrix<FloatType, Size>&    Phi,
                const Matrix<FloatType, Size, SizeQ>&   G,
                const DiagonalMatrix<FloatType, SizeQ>& Q);

  /// \brief Calculates P + c*x*x', also known as Agee Turner Rank-1 update
  /// \param[in] c  Signed scalar, c<0: downdate, c>0 update
  /// \param[in] x  A vector defining the outer product x*x' to update the matrix 
  void rank1Update(const FloatType c, const Vector<FloatType, Size>& x);

  /// \brief Set the variance at (idx,idx) and clears any correlations
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setVariance(const sint32 idx, const FloatType val);

  /// \brief Fill the covariance with first N=SrcCount rows and cols of the other covariance
  /// \tparam SrcSize   Size of the other covariance
  /// \tparam SrcCount  Count rows/cols to copy from other
  /// \param[in] other  The other matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount>
  void fill(const CovarianceMatrixFactored<FloatType, SrcSize>& other);

  /// \brief Set the Diagonal matrix element to given value
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setDiagonal(const sint32 idx, const FloatType val);

  void print() const { this->operator()().print(); }

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  TriangularMatrix<FloatType, Size, false> _u{};
  DiagonalMatrix<FloatType, Size>          _d{};
  bool                                     _isInverse{false};
};

} // namespace math
} // namespace tracking

#endif // F9044FD7_A3A8_43F4_BDD6_F43011384722
