#ifndef C50A72AB_25E1_45C6_93E5_6607B9D345E3
#define C50A72AB_25E1_45C6_93E5_6607B9D345E3

#include "base/first_include.h"
#include "base/interface_contract.h"
#include "base/require_copy_intf.h"
#include "base/require_move_intf.h"
#include <type_traits>


namespace tracking
{
namespace math
{
namespace contract
{
// TODO(matthias): fix all the macros in interface_contract.h to apply there the namespace correctly
using namespace tracking::base::contract;

template <typename ImplType>
struct CovarianceMatrixIntf
    : public base::contract::RequireCopyIntf<ImplType>
    , public base::contract::RequireMoveIntf<ImplType>
{
  CovarianceMatrixIntf()
      : base::contract::RequireCopyIntf<ImplType>()
      , base::contract::RequireMoveIntf<ImplType>()

  {
    using base::contract::has_round_brackets_const_op_int_int;
    using base::contract::has_square_brackets_const_op_int;

    static_assert(std::is_floating_point<typename ImplType::value_type>());    
    static_assert(has_member_func_Inverse<ImplType>(), ERR_MSG_MISSING_FUNCTION);

    // operator()(row,col)
    static_assert(has_round_brackets_const_op_int_int<ImplType, typename ImplType::value_type, true>::value, ERR_MSG_MISSING_FUNCTION);
    static_assert(!has_round_brackets_const_op_int_int<ImplType, typename ImplType::value_type, false>::value, ERR_MSG_MISSING_FUNCTION);
    
    // operator[](idx)
    static_assert(!has_square_brackets_const_op_int<ImplType, typename ImplType::value_type, true>::value, ERR_MSG_MISSING_FUNCTION);
    static_assert(!has_square_brackets_const_op_int<ImplType, typename ImplType::value_type, false>::value, ERR_MSG_MISSING_FUNCTION);

    //auto Identity() -> CovarianceMatrixFactored<FloatType, Size>;
  }

  CREATE_MEMBER_FUNC_SIG_CHECK(inverse, ImplType(ImplType::*)() const, Inverse );
};

} // namespace contract
} // namespace math
} // namespace tracking

#endif // C50A72AB_25E1_45C6_93E5_6607B9D345E3
