#ifndef AA87CB1E_24B7_405C_96A5_3305B5FF7DB9
#define AA87CB1E_24B7_405C_96A5_3305B5FF7DB9

#include <type_traits>

namespace tracking
{
namespace base
{
namespace contract
{

template <typename ImplType>
class RequireAbstractIntf
{
public:
  RequireAbstractIntf()
  {
    static_assert(std::is_abstract<ImplType>::value, "class is not pure virtual");
    static_assert(std::has_virtual_destructor<ImplType>::value, "destructor has to be virtual");
  }
};

} // namespace contract
} // namespace base
} // namespace tracking

#endif // AA87CB1E_24B7_405C_96A5_3305B5FF7DB9
