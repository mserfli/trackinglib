#ifndef F9044FD7_A3A8_43F4_BDD6_F43011384722
#define F9044FD7_A3A8_43F4_BDD6_F43011384722

#include "base/first_include.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/triangular_matrix.h"

namespace tracking
{
namespace math
{

// TODO(matthias): add interface contract
template <typename FloatType, sint32 Size>
class CovarianceMatrixFactored
{
public:
  // rule of 5 declarations
  CovarianceMatrixFactored() = default;
  CovarianceMatrixFactored(const CovarianceMatrixFactored<FloatType, Size>&) = default;
  CovarianceMatrixFactored(CovarianceMatrixFactored<FloatType, Size>&&) noexcept = default;
  auto operator=(const CovarianceMatrixFactored<FloatType, Size>&) -> CovarianceMatrixFactored<FloatType, Size>& = default;
  auto operator=(CovarianceMatrixFactored<FloatType, Size>&&) noexcept -> CovarianceMatrixFactored<FloatType, Size>& = default;

  explicit CovarianceMatrixFactored(const SquareMatrix<FloatType, Size>& other);
  explicit CovarianceMatrixFactored(const TriangularMatrix<FloatType, Size, false>& u,
                                    const DiagonalMatrix<FloatType, Size>&          d,
                                    const bool                                      isInverse = false);

  /// \brief Creates the composed covariance
  /// \return CovarianceMatrixFull<FloatType, Size> 
  auto compose() const -> CovarianceMatrixFull<FloatType, Size>;

  /// \brief Calculates the inverse
  /// \return CovarianceMatrixFactored<FloatType, Size>
  auto inverse() const -> CovarianceMatrixFactored<FloatType, Size>;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  TriangularMatrix<FloatType, Size, false> _u{};
  DiagonalMatrix<FloatType, Size>          _d{};
  bool                                     _isInverse{false};
};

template <typename FloatType, sint32 Size>
CovarianceMatrixFactored<FloatType, Size>::CovarianceMatrixFactored(const SquareMatrix<FloatType, Size>& other)
{
  other.decomposeUDUT(_u, _d);
}

template <typename FloatType, sint32 Size>
CovarianceMatrixFactored<FloatType, Size>::CovarianceMatrixFactored(const TriangularMatrix<FloatType, Size, false>& u,
                                                                    const DiagonalMatrix<FloatType, Size>&          d,
                                                                    const bool                                      isInverse)
    : _u{u}
    , _d{d}
    , _isInverse(isInverse)
{
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::compose() const -> CovarianceMatrixFull<FloatType, Size>
{
  CovarianceMatrixFull<FloatType, Size> cov{};
  if(_isInverse)
  {
    cov = _u.transpose() * _d * _u;
  }
  else {
    cov = _u * _d * _u.transpose();
  }
  return cov;
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::inverse() const -> CovarianceMatrixFactored<FloatType, Size>
{
  return CovarianceMatrixFactored<FloatType, Size>{_u.inverse(), _d.inverse(), true};
}

} // namespace math
} // namespace tracking

#endif // F9044FD7_A3A8_43F4_BDD6_F43011384722
