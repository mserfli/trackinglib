#ifndef B445624A_A184_4F7B_A76B_C2CEF55E0C13
#define B445624A_A184_4F7B_A76B_C2CEF55E0C13

#include <type_traits> // IWYU pragma: keep
#if __cplusplus == 202002L
#include <concepts> // IWYU pragma: keep
#endif


namespace tracking
{
namespace base
{
namespace contract
{

#define ERR_MSG_MISSING_FUNCTION "missing function"
#define ERR_MSG_DEFINED_UNEXPECTED_FUNCTION "found unexpected function definition"

// Curiously_recurring_template_pattern
// https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern

} // namespace contract
} // namespace base
} // namespace tracking

#endif // B445624A_A184_4F7B_A76B_C2CEF55E0C13
