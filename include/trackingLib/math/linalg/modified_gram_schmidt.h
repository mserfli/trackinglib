#ifndef EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
#define EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA

#include "base/first_include.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/matrix.h"
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
  /// \brief Solve inplace Phi*UDU'*Phi' on u, d matrices, or Phi*U'DU*Phi' for transposeU==true
  /// \param[in,out] u
  /// \param[in,out] d
  /// \param[in]     Phi
  /// \param[in,out] transposeU
  static void run(TriangularMatrix<FloatType, Size, false>& u,
                  DiagonalMatrix<FloatType, Size>&          d,
                  const SquareMatrix<FloatType, Size>&      Phi,
                  const bool                                transposeU);

  /// \brief Solve inplace Phi*UDU'*Phi' + G*Q*G' on u, d matrices
  /// \tparam SizeQ
  /// \param[in,out] u
  /// \param[in,out] d
  /// \param[in,out] Phi
  /// \param[in,out] G
  /// \param[in,out] Q
  template <sint32 SizeQ>
  static void run(TriangularMatrix<FloatType, Size, false>& u,
                  DiagonalMatrix<FloatType, Size>&          d,
                  const SquareMatrix<FloatType, Size>&      Phi,
                  const Matrix<FloatType, Size, SizeQ>&     G,
                  const DiagonalMatrix<FloatType, SizeQ>&   Q);
};

} // namespace math
} // namespace tracking

#endif // EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
