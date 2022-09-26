#ifndef F9044FD7_A3A8_43F4_BDD6_F43011384722
#define F9044FD7_A3A8_43F4_BDD6_F43011384722

#include "base/diagonal_matrix.h"
#include "base/matrix.h"
#include "base/square_matrix.h"
#include "base/triangular_matrix.h"

namespace tracking
{
namespace base
{

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
  explicit CovarianceMatrixFactored(const TriangularMatrix<FloatType, Size, false>& u, const DiagonalMatrix<FloatType, Size>& d);

private:
  TriangularMatrix<FloatType, Size, false> _u{};
  DiagonalMatrix<FloatType, Size>          _d{};
};

} // namespace base
} // namespace tracking

#endif // F9044FD7_A3A8_43F4_BDD6_F43011384722
