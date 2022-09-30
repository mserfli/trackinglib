#ifndef FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F
#define FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F

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

#if __cplusplus == 202002L
// clang-format off
template<typename T>
concept has_setOnes_member_func = requires {
  { std::declval<T>().setOnes() } -> std::same_as<void>;
};
template<typename T>
concept has_ones_static_member_func = requires {
  { T::ones() } -> std::same_as<T>;
};
template<typename T>
concept has_setZeros_member_func = requires {
  { std::declval<T>().setZeros() } -> std::same_as<void>;
};
template<typename T>
concept has_zeros_static_member_func = requires {
  { T::zeros() } -> std::same_as<T>;
};
template<typename T>
concept has_transpose_member_func = requires {
  { std::declval<T>().transpose() } -> std::same_as<typename T::transpose_type>;
};
template<typename T>
concept has_round_brackets_const_op_int_int = requires {
  { std::declval<const T>().operator()(std::declval<int>(), std::declval<int>()) } -> std::same_as<typename T::value_type>;
};
template<typename T>
concept has_round_brackets_op_int_int = requires {
  { std::declval<T>().operator()(std::declval<int>(), std::declval<int>()) } -> std::same_as<typename T::value_type&>;
};
template<typename T, template<typename FloatType_, sint32 Rows_, sint32 Cols_> class ClassName, sint32 Cols2>
concept has_mul_assign_op = requires {
  { std::declval<T>().operator*=(std::declval<ClassName<typename T::value_type, T::cols, Cols2>>()) } 
  -> std::same_as<ClassName<typename T::value_type, T::rows, Cols2>>;
};
// clang-format on
#endif

template <typename ImplType, template <typename FloatType_, sint32 Rows_, sint32 Cols_> class ClassName>
struct MatrixIntf
    : public base::contract::RequireCopyIntf<ImplType>
    , public base::contract::RequireMoveIntf<ImplType>
{
  MatrixIntf()
      : base::contract::RequireCopyIntf<ImplType>()
      , base::contract::RequireMoveIntf<ImplType>()

  {
    static_assert(std::is_floating_point<typename ImplType::value_type>());

#if __cplusplus == 202002L
    static_assert(has_setOnes_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_ones_static_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_setZeros_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_zeros_static_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_transpose_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_round_brackets_const_op_int_int<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_round_brackets_op_int_int<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_mul_assign_op<ImplType, ClassName, ImplType::rows - 1>, ERR_MSG_MISSING_FUNCTION);
#else
    static_assert(has_member_func_setZero<ImplType>::value, ERR_MSG_MISSING_FUNCTION);
    static_assert(has_member_func_setOnes<ImplType>::value, ERR_MSG_MISSING_FUNCTION);
#endif
  }

  CREATE_MEMBER_FUNC_SIG_CHECK(setZero, void (ImplType::*)(), setZero);
  CREATE_MEMBER_FUNC_SIG_CHECK(setOnes, void (ImplType::*)(), setOnes);
  // CREATE_MEMBER_FUNC_SIG_CHECK(transpose, typename ImplType::self(ImplType::*)(), transpose);
};

} // namespace contract
} // namespace math
} // namespace tracking

#endif // FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F
