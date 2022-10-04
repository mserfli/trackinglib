#ifndef D8B229A8_8FAA_41E5_AA06_3E1B7FBA4F47
#define D8B229A8_8FAA_41E5_AA06_3E1B7FBA4F47

#include "base/first_include.h"
#include "base/interface_contract.h"
#include "base/require_copy_intf.h"
#include "base/require_move_intf.h"
#include <type_traits>


namespace tracking
{
namespace motion
{
namespace contract
{
#if __cplusplus == 202002L
// clang-format off
namespace state_mem
{
template<typename T>
concept has_getVec_member_func = requires {
  { std::declval<const T>().getVec() } -> std::same_as<typename T::ConstStateVec&>;
};
template<typename T>
concept has_getCov_member_func = requires {
  { std::declval<const T>().getCov() } -> std::same_as<typename T::ConstStateCov&>;
};
template<typename T>
concept has_round_brackets_const_op_int_int = requires {
  { std::declval<const T>().operator()(std::declval<int>(), std::declval<int>()) } -> std::same_as<typename T::value_type>;
};
template<typename T>
concept has_round_brackets_op_int_int = requires {
  { std::declval<T>().operator()(std::declval<int>(), std::declval<int>()) } -> std::same_as<typename T::value_type&>;
};
template<typename T>
concept has_square_brackets_const_op_int = requires {
  { std::declval<const T>().operator[](std::declval<int>()) } -> std::same_as<typename T::value_type>;
};
template<typename T>
concept has_square_brackets_op_int = requires {
  { std::declval<T>().operator[](std::declval<int>()) } -> std::same_as<typename T::value_type&>;
};
// clang-format on
} // namespace state_mem
#endif

template <typename ImplType>
struct StateMemIntf
    : public base::contract::RequireCopyIntf<ImplType>
    , public base::contract::RequireMoveIntf<ImplType>
{
  StateMemIntf()
      : base::contract::RequireCopyIntf<ImplType>()
      , base::contract::RequireMoveIntf<ImplType>()

  {
    static_assert(std::is_floating_point<typename ImplType::value_type>());

#if __cplusplus == 202002L
    static_assert(state_mem::has_getVec_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(state_mem::has_getCov_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(state_mem::has_square_brackets_const_op_int<ImplType>, ERR_MSG_DEFINED_UNEXPECTED_FUNCTION);
    static_assert(state_mem::has_square_brackets_op_int<ImplType>, ERR_MSG_DEFINED_UNEXPECTED_FUNCTION);
    static_assert(state_mem::has_round_brackets_const_op_int_int<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(!state_mem::has_round_brackets_op_int_int<ImplType>, ERR_MSG_DEFINED_UNEXPECTED_FUNCTION);
#endif
  }
};

} // namespace contract
} // namespace motion
} // namespace tracking


#endif // D8B229A8_8FAA_41E5_AA06_3E1B7FBA4F47
