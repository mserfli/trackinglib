#ifndef FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F
#define FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F

#include "base/interface_contract.h"
#include "base/require_copy_intf.h"

namespace tracking
{
namespace math
{
namespace contract
{

template <typename ImplType>
struct MatrixIntf: public base::contract::RequireCopyIntf<ImplType>
{
  MatrixIntf() 
  : base::contract::RequireCopyIntf<ImplType>()
  {
    static_assert(has_setZero<ImplType, void(ImplType::*)()>::value, ERR_MSG_MISSING_FUNCTION);
  }

  DECLARE_HAS_MEM_FUNC(setZero, has_setZero);
};

} // namespace contract
} // namespace math
} // namespace tracking

#endif // FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F
