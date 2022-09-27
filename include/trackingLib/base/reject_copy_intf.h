#ifndef A4DA4781_F96E_4BD4_B524_3D07F67DBD73
#define A4DA4781_F96E_4BD4_B524_3D07F67DBD73

#include <type_traits>

namespace tracking
{
namespace base
{
namespace contract
{

template <typename ImplType>
class RejectCopyIntf
{
public:
  RejectCopyIntf()
  {
    static_assert(!std::is_copy_constructible<ImplType>::value, "copy ctor not deleted");
    static_assert(!std::is_copy_assignable<ImplType>::value, "copy assignment operator not deleted");
  }
};

} // namespace contract
} // namespace base
} // namespace tracking

#endif // A4DA4781_F96E_4BD4_B524_3D07F67DBD73
