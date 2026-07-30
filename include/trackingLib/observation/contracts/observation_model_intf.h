#ifndef E3F1A2C4_5B6D_47E8_9A0F_1C2D3E4F5A6B
#define E3F1A2C4_5B6D_47E8_9A0F_1C2D3E4F5A6B


#include "base/first_include.h"      // IWYU pragma: keep
#include "base/interface_contract.h" // IWYU pragma: keep

namespace tracking
{
namespace observation
{
namespace contract
{
#if __cplusplus == 202002L
// clang-format off
namespace observation_model
{
template<typename T>
concept has_predictMeasurementSensorFrame_const_member_func = requires {
  { std::declval<const T>().predictMeasurementSensorFrame(std::declval<const typename T::StateVec&>(), std::declval<const typename T::EgoMotionType&>()) } -> std::same_as<typename T::MeasurementVec>;
};
template<typename T>
concept has_computeJacobianSensorFrame_const_member_func = requires {
  { std::declval<const T>().computeJacobianSensorFrame(std::declval<typename T::JacobianMatrix&>(), std::declval<const typename T::StateVec&>(), std::declval<const typename T::EgoMotionType&>()) } -> std::same_as<void>;
};
// clang-format on
} // namespace observation_model
#endif // __cplusplus == 202002L

/// \brief Static interface contract for concrete observation models (CRTP)
///
/// Mirrors math::contract::CovarianceMatrixIntf / motion::contract::StateMemIntf: a zero-cost CRTP
/// base whose constructor asserts that the concrete observation model satisfies the static contract
/// consumed by motion::generic::Update — the observation function h(x) and its Jacobian H = dh/dx.
/// The concept checks are C++20 only (the codebase also builds under C++17, where concepts do not
/// exist); the type/dimension checks are always active.
///
/// \tparam ImplType  The concrete observation model (CRTP), publishing value_type, StateVec,
///                   MeasurementVec, JacobianMatrix and the static constant DimZ
template <typename ImplType>
struct ObservationModelIntf
{
  ObservationModelIntf()
  {
    static_assert(std::is_floating_point<typename ImplType::value_type>());
    static_assert(ImplType::DimZ > 0);

#if __cplusplus == 202002L
    // mandatory funcs
    static_assert(observation_model::has_predictMeasurementSensorFrame_const_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
    static_assert(observation_model::has_computeJacobianSensorFrame_const_member_func<ImplType>, ERR_MSG_MISSING_FUNCTION);
#endif // __cplusplus == 202002L
  }
};

} // namespace contract
} // namespace observation
} // namespace tracking

#endif // E3F1A2C4_5B6D_47E8_9A0F_1C2D3E4F5A6B
