#ifndef B03BEBEF_0868_4245_A0B5_AD255C4514FE
#define B03BEBEF_0868_4245_A0B5_AD255C4514FE

namespace tracking
{
namespace math
{

enum class Errors
{
  divide_by_zero,           // 9 / 0 == ?
  not_integer_division,     // 5 / 2 == 2.5 (which is not an integer)
  integer_divide_overflows, // INT_MIN / -1
  matrix_not_symmetric,
  matrix_not_positive_definite,
};

} // namespace math
} // namespace tracking

#endif // B03BEBEF_0868_4245_A0B5_AD255C4514FE
