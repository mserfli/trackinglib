#ifndef DD2B2494_7486_42BA_84E2_32308E26DBBC
#define DD2B2494_7486_42BA_84E2_32308E26DBBC

#include "base/first_include.h"
#include "math/linalg/contracts/covariance_matrix_intf.h"
#include "math/linalg/square_matrix.h"

namespace tracking
{
namespace math
{

// TODO(matthias): add contract for apaT functions
template <typename FloatType, sint32 Size>
class CovarianceMatrixFull
    : public SquareMatrix<FloatType, Size>
    , public contract::CovarianceMatrixIntf<CovarianceMatrixFull<FloatType, Size>>
{
public:
  using value_type   = FloatType;
  using compose_type = CovarianceMatrixFull;

  //  rule of 5 declarations
  CovarianceMatrixFull()                                = default;
  CovarianceMatrixFull(const CovarianceMatrixFull&)     = default;
  CovarianceMatrixFull(CovarianceMatrixFull&&) noexcept = default;
  auto operator=(const CovarianceMatrixFull&) -> CovarianceMatrixFull& = default;
  auto operator=(CovarianceMatrixFull&&) noexcept -> CovarianceMatrixFull& = default;
  virtual ~CovarianceMatrixFull()                                          = default;

  /// \brief Construct a new Covariance Matrix Full< Float Type,  Size> object
  /// \param[in] other A base class object
  /// \param[in,out] isInverse
  explicit CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other, const bool isInverse = false);

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFull
  static auto Identity() -> CovarianceMatrixFull;

  /// \brief Set Identity covariance
  void setIdentity();

  /// \brief Access operator to the covariance value at (row, col)
  /// \param[in,out] row  The specified row
  /// \param[in,out] col  The specified column
  /// \return FloatType
  auto operator()(sint32 row, sint32 col) const -> FloatType;

  /// \brief Creates the "composed" covariance, although no composition is needed
  /// \return CovarianceMatrixFull
  auto operator()() const -> CovarianceMatrixFull
  {
    assert(0 && "avoid calling this function.");
    return *this;
  }

  /// \brief Calculates the inverse based on Cholesky decomposition
  /// \return CovarianceMatrixFull
  auto inverse() const -> CovarianceMatrixFull;

  /// \brief
  /// \return true
  /// \return false
  auto isInverse() const -> bool;

  /// \brief Calculate A*P*A' inplace
  /// \param[in] A
  void apaT(const SquareMatrix<FloatType, Size>& A);

  /// \brief Calculate A*P*A'
  /// \param[in] A
  auto apaT(const SquareMatrix<FloatType, Size>& A) const -> CovarianceMatrixFull;

  /// \brief Set the variance at (idx,idx) and clears any correlations
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setVariance(const sint32 idx, const FloatType val);

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  bool _isInverse{false};
};

} // namespace math
} // namespace tracking

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
