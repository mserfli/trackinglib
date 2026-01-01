#ifndef F9044FD7_A3A8_43F4_BDD6_F43011384722
#define F9044FD7_A3A8_43F4_BDD6_F43011384722

#include "base/first_include.h"                           // IWYU pragma: keep
#include "math/linalg/contracts/covariance_matrix_intf.h" // IWYU pragma: keep
#include <iostream>                                       // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class Vector; // LCOV_EXCL_LINE

template <typename FloatType_, sint32 Size_>
class CovarianceMatrixFull; // LCOV_EXCL_LINE

// TODO(matthias): add contract for apaT functions, fill, ...
template <typename FloatType_, sint32 Size_>
class CovarianceMatrixFactored //: public contract::CovarianceMatrixIntf<CovarianceMatrixFactored<FloatType_, Size_>>
{
public:
  using value_type          = FloatType_;
  using compose_type        = CovarianceMatrixFull<FloatType_, Size_>;
  static constexpr auto dim = Size_;

  // rule of 5 declarations
  CovarianceMatrixFactored()                                                       = default;
  CovarianceMatrixFactored(const CovarianceMatrixFactored&)                        = default;
  CovarianceMatrixFactored(CovarianceMatrixFactored&&) noexcept                    = default;
  auto operator=(const CovarianceMatrixFactored&) -> CovarianceMatrixFactored&     = default;
  auto operator=(CovarianceMatrixFactored&&) noexcept -> CovarianceMatrixFactored& = default;
  virtual ~CovarianceMatrixFactored()                                              = default;

  /// \brief Construct a new Covariance Matrix Factored object
  /// \param[in] u   Unit upper triangular matrix
  /// \param[in] d   Diagonal matrix
  explicit CovarianceMatrixFactored(const TriangularMatrix<FloatType_, Size_, false, true>& u,
                                    const DiagonalMatrix<FloatType_, Size_>&                d);

  /// \brief Construct a new Covariance Matrix Factored object with initializer list representing the memory layout of the matrix
  /// \param[in] u  An initializer list describing the memory layout of the unit upper triangular matrix
  /// \param[in] d  An initializer list describing the memory layout of the diagonal matrix
  static auto FromList(const std::initializer_list<std::initializer_list<value_type>>& u,
                       const std::initializer_list<value_type>&                        d) -> CovarianceMatrixFactored;

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFactored
  static auto Identity() -> CovarianceMatrixFactored;

  /// \brief Set Identity covariance
  void setIdentity();

  /// \brief Access operator to the covariance value at (row, col)
  /// \param[in,out] row  The specified row
  /// \param[in,out] col  The specified column
  /// \return tl::expected<ValueType_, Errors>   either the value at (row,col) or an Error descriptor
  auto operator()(sint32 row, sint32 col) const -> tl::expected<value_type, Errors>;

  /// \brief Creates the composed covariance
  /// \return compose_type
  auto operator()() const -> compose_type;

  /// \brief Calculates the inverse
  /// \return tl::expected<CovarianceMatrixFactored, Errors>
  auto inverse() const -> tl::expected<CovarianceMatrixFactored, Errors>;

  /// \brief Calculates the inverse as composed covariance
  /// \return compose_type
  auto composed_inverse() const -> compose_type;

  /// \brief Calculate A*P*A' inplace
  /// \param[in] A   A square matrix which is transforming P in same space
  template <bool IsRowMajor_>
  void apaT(const SquareMatrix<FloatType_, Size_, IsRowMajor_>& A);

  /// \brief Calculate A*P*A'
  /// \param[in] A   A square matrix which is transforming P in same space
  /// \return Calculation result as new covariance matrix
  template <bool IsRowMajor_>
  auto apaT(const SquareMatrix<FloatType_, Size_, IsRowMajor_>& A) const -> CovarianceMatrixFactored;

  /// \brief Calculate Phi*P*Phi' + G*Q*G', also known as Thornton update
  /// \tparam SizeQ
  /// \param[in] Phi  A square matrix which is transforming P in same space
  /// \param[in] G    A matrix to transform Q into matrix equally sized to Phi
  /// \param[in] Q    A diagonal matrix
  template <sint32 SizeQ_>
  void thornton(const SquareMatrix<FloatType_, Size_, true>&   Phi,
                const Matrix<FloatType_, Size_, SizeQ_, true>& G,
                const DiagonalMatrix<FloatType_, SizeQ_>&      Q);

  /// \brief Calculates P + c*x*x', also known as Agee Turner Rank-1 update
  /// \param[in] c  Signed scalar, c<0: downdate, c>0 update
  /// \param[in] x  A vector defining the outer product x*x' to update the matrix
  void rank1Update(const FloatType_ c, const Vector<FloatType_, Size_>& x);

  /// \brief Set the variance at (idx,idx) and clears any correlations
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setVariance(const sint32 idx, const FloatType_ val);

  /// \brief Fill the covariance with first N=SrcCount rows and cols of the other covariance
  /// \tparam SrcSize   Size of the other covariance
  /// \tparam SrcCount  Count rows/cols to copy from other
  /// \param[in] other  The other matrix to copy from
  template <sint32 SrcSize_, sint32 SrcCount_>
  void fill(const CovarianceMatrixFactored<FloatType_, SrcSize_>& other);

  /// \brief Set the Diagonal matrix element to given value
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setDiagonal(const sint32 idx, const FloatType_ val);


  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Unsafe element read-only access
  /// \param[in] row  row of the element to read
  /// \param[in] col  column of the element to read
  /// \return ValueType_   the value at (row,col)
  [[nodiscard]] auto at_unsafe(sint32 row, sint32 col) const -> FloatType_;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief Construct a new Covariance Matrix Factored object with given initializer list representing a full covariance matrix
  /// \param[in] list  An initializer list describing a full covariance matrix
  static auto FromList(const std::initializer_list<std::initializer_list<value_type>>& list) -> CovarianceMatrixFactored;

  TriangularMatrix<FloatType_, Size_, false, true> _u{};
  DiagonalMatrix<FloatType_, Size_>                _d{};
};

} // namespace math
} // namespace tracking

#endif // F9044FD7_A3A8_43F4_BDD6_F43011384722
