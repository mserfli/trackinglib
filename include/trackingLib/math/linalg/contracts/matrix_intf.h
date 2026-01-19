#ifndef FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F
#define FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F

#if __cplusplus == 202002L
#include "base/first_include.h"      // IWYU pragma: keep
#include "base/interface_contract.h" // IWYU pragma: keep
#include "base/require_copy_intf.h"
#include "base/require_move_intf.h"
#include "math/linalg/errors.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace contract
{
// clang-format off
namespace matrix
{
template<typename T>
concept has_setOnes_member_func = requires {
  { std::declval<T>().setOnes() } -> std::same_as<void>;
};
template<typename T>
concept has_ones_static_member_func = requires {
  { T::Ones() } -> std::same_as<T>;
};
template<typename T>
concept has_setZeros_member_func = requires {
  { std::declval<T>().setZeros() } -> std::same_as<void>;
};
template<typename T>
concept has_zeros_static_member_func = requires {
  { T::Zeros() } -> std::same_as<T>;
};
template<typename T>
concept has_transpose_member_func = requires {
  { std::declval<T>().transpose() } -> std::same_as<typename T::transpose_type&>;
};
template<typename T>
concept has_round_brackets_const_op_int_int = requires {
  { std::declval<const T>().operator()(std::declval<int>(), std::declval<int>()) }
  -> std::same_as<tl::expected<typename T::value_type, Errors>>;
};
template<typename T>
concept has_round_brackets_op_int_int = requires {
  { std::declval<T>().operator()(std::declval<int>(), std::declval<int>()) }
  -> std::same_as<tl::expected<std::reference_wrapper<typename T::value_type>, Errors>>;
};
}
// clang-format on

template <typename ImplType, template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_> class ClassName>
struct MatrixIntf
    : public base::contract::RequireCopyIntf<ImplType>
    , public base::contract::RequireMoveIntf<ImplType>
{
  MatrixIntf()
      : base::contract::RequireCopyIntf<ImplType>()
      , base::contract::RequireMoveIntf<ImplType>()

  {
    static_assert(std::is_floating_point<typename ImplType::value_type>() || std::is_integral<typename ImplType::value_type>());
    static_assert(ImplType::Rows > 0);
    static_assert(ImplType::Cols > 0);

    static_assert(matrix::has_setOnes_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(matrix::has_ones_static_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(matrix::has_setZeros_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(matrix::has_zeros_static_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);

    // Conditionally check for non-Vector types
    if constexpr (ImplType::Cols > 1) // Vector has Cols == 1
    {
      static_assert(matrix::has_round_brackets_const_op_int_int<ImplType>, ERR_MSG_MISSING_FUNCTION);
      static_assert(matrix::has_round_brackets_op_int_int<ImplType>, ERR_MSG_MISSING_FUNCTION);
      static_assert(matrix::has_transpose_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    }
  }
};

} // namespace contract
} // namespace math
} // namespace tracking

#endif //__cplusplus == 202002L

#endif // FEFAD5F6_9902_4DB6_B0B1_0F8AC3FE433F
