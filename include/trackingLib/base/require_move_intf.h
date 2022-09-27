#ifndef EABA488B_84AE_4A04_8CDD_C6E89938F008
#define EABA488B_84AE_4A04_8CDD_C6E89938F008

#include <type_traits>

namespace tracking
{
namespace base
{
namespace contract
{

template <typename ImplType>
class RequireMoveIntf
{
public:
  RequireMoveIntf()
  {
    static_assert(std::is_nothrow_move_constructible<ImplType>::value, "missing noexcept move ctor");
    static_assert(std::is_nothrow_move_assignable<ImplType>::value, "missing noexcept move assignment operator");
  }
};

} // namespace contract
} // namespace base
} // namespace tracking


#endif // EABA488B_84AE_4A04_8CDD_C6E89938F008
