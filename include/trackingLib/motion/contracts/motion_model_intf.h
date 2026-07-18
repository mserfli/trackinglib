#ifndef B7C8D9E0_1F2A_43B4_85C6_7D8E9F0A1B2C
#define B7C8D9E0_1F2A_43B4_85C6_7D8E9F0A1B2C


#include "base/first_include.h"      // IWYU pragma: keep
#include "base/interface_contract.h" // IWYU pragma: keep

namespace tracking
{
namespace motion
{
namespace contract
{
#if __cplusplus == 202002L
// clang-format off
namespace motion_model
{
template<typename T>
concept has_computeEgoMotionCompensationMatrices_member_func = requires {
  { std::declval<T>().computeEgoMotionCompensationMatrices(std::declval<typename T::EgoMotionMappingMatrix&>(),
                                                           std::declval<typename T::StateMatrix&>(),
                                                           std::declval<const typename T::EgoMotionType&>()) } -> std::same_as<void>;
};
template<typename T>
concept has_compensateState_member_func = requires {
  { std::declval<T>().compensateState(std::declval<const typename T::EgoMotionType&>()) } -> std::same_as<void>;
};
template<typename T>
concept has_computeA_const_member_func = requires {
  { std::declval<const T>().computeA(std::declval<typename T::StateMatrix&>(), std::declval<typename T::value_type>()) } -> std::same_as<void>;
};
template<typename T>
concept has_applyProcessModel_member_func = requires {
  { std::declval<T>().applyProcessModel(std::declval<typename T::value_type>()) } -> std::same_as<void>;
};
template<typename T>
concept has_computeQ_static_member_func = requires {
  { T::computeQ(std::declval<typename T::ProcessNoiseDiagMatrix&>(), std::declval<typename T::value_type>()) } -> std::same_as<void>;
};
template<typename T>
concept has_computeG_static_member_func = requires {
  { T::computeG(std::declval<typename T::ProcessNoiseMappingMatrix&>(), std::declval<typename T::value_type>()) } -> std::same_as<void>;
};
// clang-format on
} // namespace motion_model
#endif // __cplusplus == 202002L

/// \brief Static interface contract for concrete motion models (CRTP)
///
/// Mirrors math::contract::CovarianceMatrixIntf / contract::StateMemIntf: a zero-cost CRTP base
/// whose constructor asserts that the concrete motion model provides the static prediction hooks
/// consumed by generic::PredictCommon / generic::Predict — ego-motion compensation, the process
/// model, the state-transition Jacobian A, and the process noise matrices Q and G. The concept
/// checks are C++20 only (the codebase also builds under C++17, where concepts do not exist); the
/// type/dimension checks are always active.
///
/// \tparam ImplType  The concrete motion model (CRTP), publishing value_type, StateMatrix,
///                   EgoMotionMappingMatrix, EgoMotionType, ProcessNoiseDiagMatrix,
///                   ProcessNoiseMappingMatrix and the static constant NUM_STATE_VARIABLES
template <typename ImplType>
struct MotionModelIntf
{
  MotionModelIntf()
  {
    static_assert(std::is_floating_point<typename ImplType::value_type>());
    static_assert(ImplType::NUM_STATE_VARIABLES > 0);

#if __cplusplus == 202002L
    // mandatory funcs
    static_assert(motion_model::has_computeEgoMotionCompensationMatrices_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(motion_model::has_compensateState_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(motion_model::has_computeA_const_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(motion_model::has_applyProcessModel_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(motion_model::has_computeQ_static_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(motion_model::has_computeG_static_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
#endif // __cplusplus == 202002L
  }
};

} // namespace contract
} // namespace motion
} // namespace tracking

#endif // B7C8D9E0_1F2A_43B4_85C6_7D8E9F0A1B2C
