#ifndef DD2B2494_7486_42BA_84E2_32308E26DBBC
#define DD2B2494_7486_42BA_84E2_32308E26DBBC

#include "base/first_include.h"                           // IWYU pragma: keep
#include "math/linalg/contracts/covariance_matrix_intf.h" // IWYU pragma: keep
#include "math/linalg/square_matrix.h"

namespace tracking
{
namespace math
{

template <typename FloatType_, sint32 Size_>
class CovarianceMatrixFull: public SquareMatrix<FloatType_, Size_, true>
//, public contract::CovarianceMatrixIntf<CovarianceMatrixFull<FloatType_, Size_, IsRowMajor_>>
{
public:
  using SquareMatrix = SquareMatrix<FloatType_, Size_, true>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using SquareMatrix::SquareMatrix;

  using value_type          = FloatType_;
  using compose_type        = CovarianceMatrixFull;
  static constexpr auto dim = Size_;

  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a new Covariance Matrix Full< Float Type,  Size_> object
  /// \param[in] other A base class object
  explicit CovarianceMatrixFull(const SquareMatrix& other)
      : SquareMatrix{other}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Move construct a new Covariance Matrix Full< Float Type,  Size_> object
  /// \param[in] other A base class object
  explicit CovarianceMatrixFull(SquareMatrix&& other) noexcept
      : SquareMatrix{std::forward<SquareMatrix>(other)}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Construct a new Covariance Matrix Full< Float Type,  Size_> object from a transposed SquareMatrix
  /// \param[in] other A transposed base class object
  explicit CovarianceMatrixFull(const SquareMatrix::transpose_type& other)
      : SquareMatrix{other.transpose()}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Move construct a new Covariance Matrix Full< Float Type,  Size_> object from a transposed SquareMatrix
  /// \param[in] other A transposed base class object
  explicit CovarianceMatrixFull(SquareMatrix::transpose_type&& other) noexcept
      : SquareMatrix{std::move(other).transpose_rvalue()}
  {
    assert(this->isSymmetric() && "Constructed covariance not symmetric");
  }

  /// \brief Construct a new Covariance Matrix Full object with given initializer list representing the memory layout of the
  /// matrix
  /// \param[in] list  An initializer list describing the memory layout of the matrix
  static auto FromList(const std::initializer_list<std::initializer_list<value_type>>& list) -> CovarianceMatrixFull
  {
    return CovarianceMatrixFull{SquareMatrix::FromList(list)};
  }

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFull  Resulting identity matrix
  static auto Identity() -> CovarianceMatrixFull { return CovarianceMatrixFull{SquareMatrix::Identity()}; }

  /// \brief Set internal matrix to the Identity matrix
  void setIdentity() { SquareMatrix::setIdentity(); }

  /// \brief Access operator to the covariance value at (row, col)
  using SquareMatrix::operator();

  /// \brief Creates the "composed" covariance, although no composition is needed
  /// \return const CovarianceMatrixFull&
  auto operator()() const -> const CovarianceMatrixFull& { return *this; }

  /// \brief Calculates the inverse based on Cholesky decomposition
  /// \return tl::expected<CovarianceMatrixFull, Errors>
  auto inverse() const -> tl::expected<CovarianceMatrixFull, Errors>;

  /// \brief Checks inverse status
  /// \return true
  /// \return false
  [[nodiscard]] auto isInverse() const -> bool { return _isInverse; }

  /// \brief Calculate A*P*A' inplace
  /// \param[in] A
  template <bool IsRowMajor_>
  void apaT(const tracking::math::SquareMatrix<FloatType_, Size_, IsRowMajor_>& A);

  /// \brief Calculate A*P*A'
  /// \param[in] A
  template <bool IsRowMajor_>
  auto apaT(const tracking::math::SquareMatrix<FloatType_, Size_, IsRowMajor_>& A) const -> CovarianceMatrixFull;

  /// \brief Set the variance at (idx,idx) and clears any correlations
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setVariance(const sint32 idx, const FloatType_ val);

  // private:
  bool _isInverse{false};
};

} // namespace math
} // namespace tracking

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
