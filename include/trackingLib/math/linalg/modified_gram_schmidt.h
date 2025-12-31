#ifndef EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA
#define EAB4F7E8_4154_4054_AA4E_CFE8C4C007EA

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL; // LCOV_EXCL_LINE


template <typename FloatType_, sint32 Size_>
class ModifiedGramSchmidt
{
public:
  /// \brief Solve inplace Phi*UDU'*Phi' on u, d matrices
  /// \param[in,out] u
  /// \param[in,out] d
  /// \param[in]     Phi
  static void run(TriangularMatrix<FloatType_, Size_, false, true>& u,
                  DiagonalMatrix<FloatType_, Size_>&                d,
                  const SquareMatrix<FloatType_, Size_, true>&      Phi);

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
