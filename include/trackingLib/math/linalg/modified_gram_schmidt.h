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

template <typename FloatType_, sint32 Size_>
class ModifiedGramSchmidt
{
public:
  /// \brief Solve inplace Phi*UDU'*Phi' on u, d matrices, or Phi*U'DU*Phi' for transposeU==true
  /// \param[in,out] u
  /// \param[in,out] d
  /// \param[in]     Phi
  /// \param[in,out] transposeU
  static void run(TriangularMatrix<FloatType_, Size_, false, true>& u,
                  DiagonalMatrix<FloatType_, Size_>&                d,
                  const SquareMatrix<FloatType_, Size_, true>&      Phi,
                  const bool                                        transposeU);

  /// \brief Solve inplace Phi*UDU'*Phi' + G*Q*G' on u, d matrices
  /// \tparam SizeQ
  /// \param[in,out] u
  /// \param[in,out] d
  /// \param[in,out] Phi
  /// \param[in,out] G
  /// \param[in,out] Q
  template <sint32 SizeQ_>
  static void run(TriangularMatrix<FloatType_, Size_, false, true>& u,
                  DiagonalMatrix<FloatType_, Size_>&                d,
                  const SquareMatrix<FloatType_, Size_, true>&      Phi,
                  const Matrix<FloatType_, Size_, SizeQ_, true>&    G,
                  const DiagonalMatrix<FloatType_, SizeQ_>&         Q);
};

} // namespace math
} // namespace tracking

#endif // EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
