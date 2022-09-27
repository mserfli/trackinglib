#ifndef AD362419_8F56_4A82_AA4D_A026AEBAA86E
#define AD362419_8F56_4A82_AA4D_A026AEBAA86E

#include "base/atomic_types.h"

namespace tracking
{
namespace base
{
namespace contract
{

template <typename ImplType>
class RequireCopyIntf
{
public:
  RequireCopyIntf()
  {
    if (cnt == 0)
    {
      ++cnt; // prevent recursive calls
      ImplType instance{};

      // check existence of copy constructor
      ImplType copy_ctor(instance);

      // check existence of copy assignment
      ImplType  copy_asgn;
      copy_asgn.operator=(instance);
    }
  }

private:
  static sint32 cnt;
};

template <typename ImplType>
sint32 RequireCopyIntf<ImplType>::cnt = 0;

} // namespace contract
} // namespace base
} // namespace tracking

#endif // AD362419_8F56_4A82_AA4D_A026AEBAA86E
