#ifndef EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
#define EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA

#include "base/first_include.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/square_matrix.h"
#include "math/linalg/triangular_matrix.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
class ModifiedGramSchmidt
{
public:
  /// \brief Solve inplace Phi*UDU'*Phi' on u, d matrices
  /// \param[in,out] u
  /// \param[in,out] d
  /// \param[in]     phi
  static void run(TriangularMatrix<FloatType, Size, false>& u,
                  DiagonalMatrix<FloatType, Size>&          d,
                  const SquareMatrix<FloatType, Size>&      phi,
                  const bool transposeU = false);
};

} // namespace math
} // namespace tracking

#endif // EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
