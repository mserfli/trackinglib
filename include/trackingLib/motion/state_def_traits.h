#ifndef C4E8A1B2_9D3F_4C5E_A6B7_8C9D0E1F2A3B
#define C4E8A1B2_9D3F_4C5E_A6B7_8C9D0E1F2A3B

#include "base/first_include.h" // IWYU pragma: keep
#include <type_traits>

namespace tracking
{
namespace motion
{
namespace detail
{

/// \brief Detection trait for StateDefs providing a position (X and Y enumerators)
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_, typename = void>
struct has_position: std::false_type
{
};

/// \brief Specialization matching StateDefs that provide X and Y enumerators
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_>
struct has_position<StateDef_, std::void_t<decltype(StateDef_::X), decltype(StateDef_::Y)>>: std::true_type
{
};

/// \brief Detection trait for StateDefs providing a velocity (VX and VY enumerators)
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_, typename = void>
struct has_velocity: std::false_type
{
};

/// \brief Specialization matching StateDefs that provide VX and VY enumerators
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_>
struct has_velocity<StateDef_, std::void_t<decltype(StateDef_::VX), decltype(StateDef_::VY)>>: std::true_type
{
};

/// \brief Detection trait for StateDefs providing an acceleration (AX and AY enumerators)
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_, typename = void>
struct has_acceleration: std::false_type
{
};

/// \brief Specialization matching StateDefs that provide AX and AY enumerators
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_>
struct has_acceleration<StateDef_, std::void_t<decltype(StateDef_::AX), decltype(StateDef_::AY)>>: std::true_type
{
};

} // namespace detail

/// \brief true if the StateDef provides position enumerators X and Y
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_>
inline constexpr bool has_position_v = detail::has_position<StateDef_>::value;

/// \brief true if the StateDef provides velocity enumerators VX and VY
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_>
inline constexpr bool has_velocity_v = detail::has_velocity<StateDef_>::value;

/// \brief true if the StateDef provides acceleration enumerators AX and AY
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_>
inline constexpr bool has_acceleration_v = detail::has_acceleration<StateDef_>::value;

/// \brief State dimension of the StateDef (number of state variables)
/// \tparam StateDef_ State definition structure containing state variable enumeration
template <typename StateDef_>
inline constexpr sint32 state_dimension_v = StateDef_::NUM_STATE_VARIABLES;

} // namespace motion
} // namespace tracking

#endif // C4E8A1B2_9D3F_4C5E_A6B7_8C9D0E1F2A3B
