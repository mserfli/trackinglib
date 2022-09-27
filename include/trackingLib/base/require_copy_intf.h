#ifndef AD362419_8F56_4A82_AA4D_A026AEBAA86E
#define AD362419_8F56_4A82_AA4D_A026AEBAA86E

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
    static_assert(std::is_copy_constructible<ImplType>::value, "missing copy ctor");
    static_assert(std::is_copy_assignable<ImplType>::value, "missing copy assignment operator");
  }
};

} // namespace contract
} // namespace base
} // namespace tracking

#endif // AD362419_8F56_4A82_AA4D_A026AEBAA86E
