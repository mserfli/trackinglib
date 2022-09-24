#ifndef DD2B2494_7486_42BA_84E2_32308E26DBBC
#define DD2B2494_7486_42BA_84E2_32308E26DBBC

#include "base/matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size>
class CovarianceMatrixFull: public SquareMatrix<FloatType, Size>
{
public:
  using FullMatrix = SquareMatrix<FloatType, Size>;

  CovarianceMatrixFull<FloatType, Size>() = default;
  
  // NOLINTNEXTLINE(google-explicit-constructor)
  CovarianceMatrixFull<FloatType, Size>(
      const SquareMatrix<FloatType, Size>& other);
};

template <typename FloatType, sint32 Size>
CovarianceMatrixFull<FloatType, Size>::CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{other}
{
}

} // namespace base
} // namespace tracking

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
