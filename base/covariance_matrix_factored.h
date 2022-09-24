#ifndef F9044FD7_A3A8_43F4_BDD6_F43011384722
#define F9044FD7_A3A8_43F4_BDD6_F43011384722

#include "base/matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size>
class CovarianceMatrixFactored
{
public:
  static constexpr sint32 rows = Size;
  using const_type             = std::add_const<FloatType>;
  using type                   = std::remove_const<FloatType>;
  using self                   = CovarianceMatrixFactored<type, rows>;
  using FullMatrix             = SquareMatrix<type, rows>;
};

} // namespace base
} // namespace tracking

#endif // F9044FD7_A3A8_43F4_BDD6_F43011384722
