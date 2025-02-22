#ifndef C50A72AB_25E1_45C6_93E5_6607B9D345E3
#define C50A72AB_25E1_45C6_93E5_6607B9D345E3

#include "base/first_include.h" // IWYU pragma: keep
#include "base/interface_contract.h"
#include "base/require_copy_intf.h"
#include "base/require_move_intf.h"
#include "math/linalg/square_matrix.h"

#if __cplusplus == 202002L
#include <concepts>
#endif
#include <type_traits>

namespace tracking
{
namespace math
{
namespace contract
{
#if __cplusplus == 202002L
// clang-format off
namespace covariance
{
template<typename T>
concept has_identity_static_member_func = requires {
  { std::declval<T>().Identity() } -> std::same_as<T>;
};
template<typename T>
concept has_setIdentity_member_func = requires {
  { std::declval<T>().setIdentity() } -> std::same_as<void>;
};
template<typename T>
concept has_setVariance_member_func = requires {
  { std::declval<T>().setVariance(std::declval<int>(), std::declval<int>()) } -> std::same_as<void>;
};
template<typename T>
concept has_inverse_member_func = requires {
  { std::declval<const T>().inverse() } -> std::same_as<tl::expected<T, Errors>>;
};
template<typename T>
concept has_isInverse_member_func = requires {
  { std::declval<const T>().isInverse() } -> std::same_as<bool>;
};
template<typename T>
concept has_apaT_member_func = requires {
  { std::declval<const T>().apaT(std::declval<SquareMatrix<typename T::value_type, T::dim, T::row_major>>()) } -> std::same_as<T>;
};
template<typename T>
concept has_apaT_inplace_member_func = requires {
  { std::declval<T>().apaT(std::declval<SquareMatrix<typename T::value_type, T::dim, T::row_major>>()) } -> std::same_as<void>;
};
template<typename T>
concept has_print_member_func = requires {
  { std::declval<const T>().print() } -> std::same_as<void>;
};
template<typename T>
concept has_round_brackets_op = requires {
  { std::declval<T>().operator()() } -> std::convertible_to<typename T::compose_type&>;
};
template<typename T>
concept has_round_brackets_const_op = requires {
  { std::declval<const T>().operator()() } -> std::convertible_to<typename T::compose_type>; // allow const reference as well as plain type
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
} // namespace covariance
#endif

template <typename ImplType>
struct CovarianceMatrixIntf
    : public base::contract::RequireCopyIntf<ImplType>
    , public base::contract::RequireMoveIntf<ImplType>
{
  CovarianceMatrixIntf()
      : base::contract::RequireCopyIntf<ImplType>()
      , base::contract::RequireMoveIntf<ImplType>()
  {
    static_assert(std::is_floating_point<typename ImplType::value_type>());

#if __cplusplus == 202002L
    static_assert(covariance::has_identity_static_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_setIdentity_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_setVariance_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_inverse_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_isInverse_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_apaT_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_apaT_inplace_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_print_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);

    static_assert(covariance::has_round_brackets_const_op<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(covariance::has_round_brackets_const_op_int_int<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(!covariance::has_round_brackets_op<ImplType>, ERR_MSG_DEFINED_UNEXPECTED_FUNCTION);
    static_assert(!covariance::has_round_brackets_op_int_int<ImplType>, ERR_MSG_DEFINED_UNEXPECTED_FUNCTION);
    static_assert(!covariance::has_square_brackets_const_op_int<ImplType>, ERR_MSG_DEFINED_UNEXPECTED_FUNCTION);
    static_assert(!covariance::has_square_brackets_op_int<ImplType>, ERR_MSG_DEFINED_UNEXPECTED_FUNCTION);
#endif
  }
};

} // namespace contract
} // namespace math
} // namespace tracking

#endif // C50A72AB_25E1_45C6_93E5_6607B9D345E3
