# Measurement Update Implementation Plan - Final

## Overview

This plan provides a comprehensive approach to implementing measurement updates for the trackinglib library, including an architectural refactoring step to improve consistency between prediction and update patterns.

## Step 0: Architectural Refactoring for Consistency (COMPLETED 2026-01-25)

### Prediction Architecture Refactoring

**Objective**: Refactor the prediction architecture to match the proposed update pattern for better consistency and maintainability.

#### Current Issues:
1. **Code Duplication**: Each motion model (CV, CA) implements its own predict methods
2. **Inconsistent Pattern**: Prediction uses GenericPredict but motion models still have individual predict implementations
3. **Maintenance Burden**: Adding new motion models requires duplicating prediction logic

#### Proposed Solution:
Move prediction logic to ExtendedMotionModel following the same pattern as proposed for updates.

**Benefits**:
- **Consistency**: Prediction follows same pattern as update
- **Reduced Duplication**: No need for individual predict methods in each motion model
- **Easier Maintenance**: New motion models automatically get prediction capability
- **Better Separation**: GenericPredict handles all prediction logic

## Step 1: Information Filter Representation Fix (COMPLETED 2026-02-15)

### Problem Analysis

**Current Issue**: The information filter currently estimates the state x directly, but according to literature, in information space we should estimate y = Y * x (information vector) where Y is the information matrix (inverse of covariance P).

**Current Implementation**: In [`generic_predict_common.hpp`](include/trackingLib/motion/generic_predict_common.hpp), the state prediction is handled identically for both Kalman and Information filters:

The `applyProcessModel(dt)` call applies the state transition directly to the state vector, which is correct for Kalman Filter but incorrect for Information Filter. In information space, the state transition must be applied differently.

### Solution Design: Generic Predict handles conversion to state space and back to information space for InformationFilter predictions

see [`Prediction flow`](plans/info_filter_prediction_flow.md)

## Step 2: Observation Model Framework - C++17 Improvements

### State Def Trait System (C++17)

Create a new header file `include/trackingLib/motion/state_def_traits.h` with C++17 features:

```cpp
#ifndef TRACKINGLIB_STATE_DEF_TRAITS_H
#define TRACKINGLIB_STATE_DEF_TRAITS_H

#include "base/first_include.h"
#include <type_traits>
#include <utility>

namespace tracking
{
namespace motion
{

/// \brief Type traits for StateDef structures
/// These traits allow compile-time verification that a StateDef provides
/// the required state variables for an observation model.

namespace detail
{
    template <typename StateDef, typename = void>
    struct has_position_impl : std::false_type {};
    
    template <typename StateDef>
    struct has_position_impl<StateDef, 
        std::void_t<decltype(StateDef::X), decltype(StateDef::Y)>>
        : std::true_type {};
        
    template <typename StateDef, typename = void>
    struct has_velocity_impl : std::false_type {};
    
    template <typename StateDef>
    struct has_velocity_impl<StateDef, 
        std::void_t<decltype(StateDef::VX), decltype(StateDef::VY)>>
        : std::true_type {};
        
    template <typename StateDef, typename = void>
    struct has_acceleration_impl : std::false_type {};
    
    template <typename StateDef>
    struct has_acceleration_impl<StateDef, 
        std::void_t<decltype(StateDef::AX), decltype(StateDef::AY)>>
        : std::true_type {};
        
    template <typename StateDef, typename = void>
    struct has_state_size_impl : std::false_type {};
    
    template <typename StateDef>
    struct has_state_size_impl<StateDef, 
        std::void_t<decltype(StateDef::NUM_STATE_VARIABLES)>>
        : std::true_type {};
}

/// \brief Check if StateDef has position variables (X, Y)
template <typename StateDef>
inline constexpr bool has_position_v = detail::has_position_impl<StateDef>::value;

/// \brief Check if StateDef has velocity variables (VX, VY)
template <typename StateDef>
inline constexpr bool has_velocity_v = detail::has_velocity_impl<StateDef>::value;

/// \brief Check if StateDef has acceleration variables (AX, AY)
template <typename StateDef>
inline constexpr bool has_acceleration_v = detail::has_acceleration_impl<StateDef>::value;

/// \brief Check if StateDef has valid state size
template <typename StateDef>
inline constexpr bool has_state_size_v = detail::has_state_size_impl<StateDef>::value;

/// \brief Get the state dimension from StateDef
template <typename StateDef>
inline constexpr sint32 state_dimension = []() constexpr {
    static_assert(has_state_size_v<StateDef>, "StateDef must have NUM_STATE_VARIABLES");
    return StateDef::NUM_STATE_VARIABLES;
}();

/// \brief Compile-time state validation utility
template <typename StateDef>
struct state_validation
{
    static_assert(has_state_size_v<StateDef>, 
        "StateDef must define NUM_STATE_VARIABLES constant");
    static_assert(state_dimension<StateDef> > 0, 
        "State dimension must be positive");
};

/// \brief Helper concept to check if type has position variables (C++20, but can be emulated in C++17)
template <typename StateDef>
inline constexpr bool has_position = has_position_v<StateDef>;

template <typename StateDef>
inline constexpr bool has_velocity = has_velocity_v<StateDef>;

template <typename StateDef>
inline constexpr bool has_acceleration = has_acceleration_v<StateDef>;

} // namespace motion
} // namespace tracking

#endif // TRACKINGLIB_STATE_DEF_TRAITS_H
```

### IObservationModel (Pure Abstract Interface)

```cpp
template <typename CovarianceMatrixPolicy_>
class IObservationModel
    : public math::contract::CovarianceMatrixPolicyIntf<CovarianceMatrixPolicy_>
    , public base::contract::RequireAbstractIntf<IObservationModel<CovarianceMatrixPolicy_>>
{
public:
    using value_type            = typename CovarianceMatrixPolicy_::value_type;
    using CovarianceMatrixPolicy = CovarianceMatrixPolicy_;

    // rule of 5 declarations
    IObservationModel()          = default;
    virtual ~IObservationModel() = default;

    // Pure virtual methods for all observation models
    [[nodiscard]] virtual sint32 getActiveComponentCount() const noexcept = 0;
    [[nodiscard]] virtual bool isComponentSupported(sint32 index, const math::Vector<value_type, /* unknown dimension */>& state) const noexcept = 0;

    // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround to keep following idententation
  // clang-format on

  // rule of 5 declarations (remaining declarations are protected according to A12-8-6)
  IObservationModel(const IObservationModel& other)                    = default;
  IObservationModel(IObservationModel&&) noexcept                      = default;
  auto operator=(const IObservationModel& other) -> IObservationModel& = default;
  auto operator=(IObservationModel&&) noexcept -> IObservationModel&   = default;
};
```

### ExtendedObservationModel (Partial Implementation with StateMem Inheritance)

```cpp
template <typename ObservationModel_, typename ObservationModelTrait_>
class ExtendedObservationModel
    : public IObservationModel<typename ObservationModelTrait_::CovarianceMatrixPolicy>
    , public motion::StateMem<typename ObservationModelTrait_::CovarianceMatrixPolicy, ObservationModelTrait_::DimZ>
    , public base::contract::RequireAbstractIntf<ExtendedObservationModel<ObservationModel_, ObservationModelTrait_>>
{
public:
  using value_type             = typename ObservationModelTrait_::value_type;
  using CovarianceMatrixPolicy = typename ObservationModelTrait_::CovarianceMatrixPolicy;
  using BaseIObservationModel  = IObservationModel<CovarianceMatrixPolicy>;
  using BaseStateMem           = motion::StateMem<CovarianceMatrixPolicy, ObservationModelTrait_::DimZ>;
  
  using MeasurementVec         = typename BaseStateMem::StateVec;
  using MeasurementCov         = typename BaseStateMem::StateCov;
  using StateDef               = typename ObservationModelTrait_::StateDef;
  using StateVec               = math::Vector<value_type, motion::state_dimension<StateDef>>;
  using JacobianMatrix         = math::Matrix<value_type, ObservationModelTrait_::DimZ, motion::state_dimension<StateDef>>;

  // rule of 5 declarations
  ExtendedObservationModel()          = default;
  virtual ~ExtendedObservationModel() = default;

  /// \brief Create measurement vector from initializer list
  static auto MeasurementVecFromList(const std::initializer_list<value_type>& list) -> MeasurementVec { 
    return MeasurementVec::FromList(list); 
  }

  /// \brief Create measurement covariance from initializer list
  static auto MeasurementCovFromList(const std::initializer_list<std::initializer_list<value_type>>& list) -> MeasurementCov
  {
    if constexpr (CovarianceMatrixPolicy::is_factored)
    {
      return math::conversions::CovarianceMatrixFactoredFromList<value_type, ObservationModelTrait_::DimZ>(list);
    }
    else
    {
      return MeasurementCov::FromList(list);
    }
  }

  /// \brief Create factored measurement covariance from initializer list
  template <typename T = CovarianceMatrixPolicy>
  static auto MeasurementCovFromList(const std::initializer_list<std::initializer_list<value_type>>& u,
                               const std::initializer_list<value_type>& d) -> std::enable_if_t<T::is_factored, MeasurementCov>
  {
    return MeasurementCov::FromList(u, d);
  }

  /// \brief Create complete ExtendedObservationModel from initializer lists
  static auto FromLists(const std::initializer_list<value_type>&                        vecList,
                        const std::initializer_list<std::initializer_list<value_type>>& covList) -> ObservationModel_
  {
    auto vec = MeasurementVecFromList(vecList);
    auto cov = MeasurementCovFromList(covList);
    return ObservationModel_{vec, cov};
  }

  /// \brief Get the actual measurement vector z
  [[nodiscard]] auto getMeasurement() const noexcept -> const MeasurementVec& 
  { 
    return this->getVec(); 
  }
  
  /// \brief Get the measurement noise covariance R
  [[nodiscard]] auto getMeasurementCovariance() const noexcept -> const MeasurementCov& 
  { 
    return this->getCov(); 
  }
  
  /// \brief Get a single measurement component (row of z)
  [[nodiscard]] auto getComponent(sint32 index) const noexcept -> value_type 
  { 
    return this->getVec()[index]; 
  }
  
  /// \brief Get measurement covariance for a single component
  [[nodiscard]] auto getComponentCovariance(sint32 index) const noexcept -> value_type 
  { 
    return this->getCov()(index, index); 
  }

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround to keep following idententation
  // clang-format on

  // rule of 5 declarations (remaining declarations are protected according to A12-8-6)
  ExtendedObservationModel(const ExtendedObservationModel& other)                    = default;
  ExtendedObservationModel(ExtendedObservationModel&&) noexcept                      = default;
  auto operator=(const ExtendedObservationModel& other) -> ExtendedObservationModel& = default;
  auto operator=(ExtendedObservationModel&&) noexcept -> ExtendedObservationModel&   = default;

  /// \brief Protected constructor to initialize from measurement and covariance
  explicit ExtendedObservationModel(const MeasurementVec& measurement, const MeasurementCov& covariance)
      : BaseIObservationModel{}
      , BaseStateMem{measurement, covariance}
  {
    assert(covariance.determinant() > 0);
  }
};
```

### Observation Model Trait System

```cpp
template <typename FloatType_, typename StateDef_, sint32 DimZ_, typename CovarianceMatrixPolicy_ = math::FullCovarianceMatrixPolicy<FloatType_>>
struct ObservationModelTrait
{
  using value_type            = FloatType_;
  using StateDef              = StateDef_;
  using CovarianceMatrixPolicy = CovarianceMatrixPolicy_;
  static constexpr sint32 DimZ = DimZ_;
};
```

### Concrete Observation Models

```cpp
// Range-Bearing-Doppler Observation Model (NEW)
template <typename FloatType_, typename StateDef_, typename CovarianceMatrixPolicy_ = math::FullCovarianceMatrixPolicy<FloatType_>>
class RangeBearingDopplerObservationModel 
    : public ExtendedObservationModel<RangeBearingDopplerObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                      ObservationModelTrait<FloatType_, StateDef_, 3, CovarianceMatrixPolicy_>>
{
    static_assert(motion::has_position_v<StateDef_>, 
        "StateDef must have X and Y position variables");
    
    using Base = ExtendedObservationModel<RangeBearingDopplerObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                           ObservationModelTrait<FloatType_, StateDef_, 3, CovarianceMatrixPolicy_>>;
    
public:
    using typename Base::MeasurementVec;
    using typename Base::MeasurementCov;
    using typename Base::StateVec;
    using typename Base::JacobianMatrix;
    
    RangeBearingDopplerObservationModel(FloatType_ range, FloatType_ bearing, FloatType_ doppler,
                                       const MeasurementCov& covariance,
                                       FloatType_ sensor_x = 0, FloatType_ sensor_y = 0)
        : Base(MeasurementVec{range, bearing, doppler}, covariance),
          sensor_x_(sensor_x), sensor_y_(sensor_y) {}
    
    auto getActiveComponentCount() const -> sint32 override
    {
        sint32 count = 2;  // range + bearing (always supported if position exists)
        if constexpr (motion::has_velocity_v<StateDef_>) {
            count += 1;  // add doppler if velocity exists
        }
        return count;
    }
    
    auto getComponent(sint32 index) const -> FloatType_ override
    {
        return this->getVec()[index];
    }
    
    auto predictComponent(sint32 index, const StateVec& state) const -> FloatType_ override
    {
        switch (index) {
            case 0: return predictRange(state);
            case 1: return predictBearing(state);
            case 2: 
                if constexpr (motion::has_velocity_v<StateDef_>) {
                    return predictDoppler(state);
                }
                return 0.0;  // should not reach if isComponentSupported() is checked
        }
        return 0.0;
    }
    
    void computeJacobianRow(sint32 index, math::Vector<FloatType_, motion::state_dimension<StateDef_>>& row, 
                           const StateVec& state) const override
    {
        row.setZero();
        
        const auto dx = state[StateDef_::X] - sensor_x_;
        const auto dy = state[StateDef_::Y] - sensor_y_;
        const auto r2 = dx*dx + dy*dy;
        const auto r = std::sqrt(r2);
        
        switch (index) {
            case 0: // Range row
                row[StateDef_::X] = dx / r;
                row[StateDef_::Y] = dy / r;
                break;
                
            case 1: // Bearing row
                row[StateDef_::X] = -dy / r2;
                row[StateDef_::Y] = dx / r2;
                break;
                
            case 2: // Doppler row (only if velocity exists)
                if constexpr (motion::has_velocity_v<StateDef_>) {
                    const auto vx = state[StateDef_::VX];
                    const auto vy = state[StateDef_::VY];
                    
                    const auto numerator_x = vx*r2 - dx*(dx*vx + dy*vy);
                    row[StateDef_::X] = numerator_x / (r*r2);
                    
                    const auto numerator_y = vy*r2 - dy*(dx*vx + dy*vy);
                    row[StateDef_::Y] = numerator_y / (r*r2);
                    
                    row[StateDef_::VX] = dx / r;
                    row[StateDef_::VY] = dy / r;
                }
                break;
        }
    }
    
    auto getComponentCovariance(sint32 index) const -> FloatType_ override
    {
        return this->getCov()(index, index);
    }
    
    auto isComponentSupported(sint32 index, const StateVec& /*state*/) const -> bool override
    {
        switch (index) {
            case 0: return true;
            case 1: return true;
            case 2: return motion::has_velocity_v<StateDef_>;
            default: return false;
        }
    }
    
    auto predictMeasurement(const StateVec& state) const -> MeasurementVec override
    {
        const auto dx = state[StateDef_::X] - sensor_x_;
        const auto dy = state[StateDef_::Y] - sensor_y_;
        const auto vx = motion::has_velocity_v<StateDef_> ? state[StateDef_::VX] : 0.0;
        const auto vy = motion::has_velocity_v<StateDef_> ? state[StateDef_::VY] : 0.0;
        
        const auto r = std::sqrt(dx*dx + dy*dy);
        const auto theta = std::atan2(dy, dx);
        const auto v_r = (dx*vx + dy*vy) / r;  // radial velocity (Doppler)
        
        return MeasurementVec{r, theta, v_r};
    }
    
    auto computeJacobian(JacobianMatrix& H, const StateVec& state) const -> void override
    {
        H.setZero();
        
        const auto dx = state[StateDef_::X] - sensor_x_;
        const auto dy = state[StateDef_::Y] - sensor_y_;
        const auto r2 = dx*dx + dy*dy;
        const auto r = std::sqrt(r2);
        
        // ====================
        // Range Jacobian (row 0)
        // ====================
        H(0, StateDef_::X) = dx / r;
        H(0, StateDef_::Y) = dy / r;
        
        if constexpr (motion::has_velocity_v<StateDef_>) {
            H(0, StateDef_::VX) = 0.0;
            H(0, StateDef_::VY) = 0.0;
        }
        
        if constexpr (motion::has_acceleration_v<StateDef_>) {
            H(0, StateDef_::AX) = 0.0;
            H(0, StateDef_::AY) = 0.0;
        }
        
        // ====================
        // Bearing Jacobian (row 1)
        // ====================
        H(1, StateDef_::X) = -dy / r2;
        H(1, StateDef_::Y) = dx / r2;
        
        if constexpr (motion::has_velocity_v<StateDef_>) {
            H(1, StateDef_::VX) = 0.0;
            H(1, StateDef_::VY) = 0.0;
        }
        
        if constexpr (motion::has_acceleration_v<StateDef_>) {
            H(1, StateDef_::AX) = 0.0;
            H(1, StateDef_::AY) = 0.0;
        }
        
        // ====================
        // Doppler Jacobian (row 2) - ONLY if velocity exists
        // ====================
        if constexpr (motion::has_velocity_v<StateDef_>) {
            const auto vx = state[StateDef_::VX];
            const auto vy = state[StateDef_::VY];
            
            // ∂v_r/∂x = (vx*r² - dx*(dx*vx + dy*vy))/r³
            const auto numerator_x = vx*r2 - dx*(dx*vx + dy*vy);
            H(2, StateDef_::X) = numerator_x / (r*r2);
            
            // ∂v_r/∂y = (vy*r² - dy*(dx*vx + dy*vy))/r³
            const auto numerator_y = vy*r2 - dy*(dx*vx + dy*vy);
            H(2, StateDef_::Y) = numerator_y / (r*r2);
            
            // ∂v_r/∂vx = dx / r
            H(2, StateDef_::VX) = dx / r;
            
            // ∂v_r/∂vy = dy / r
            H(2, StateDef_::VY) = dy / r;
            
            if constexpr (motion::has_acceleration_v<StateDef_>) {
                H(2, StateDef_::AX) = 0.0;
                H(2, StateDef_::AY) = 0.0;
            }
        }
    }
    
private:
    auto predictRange(const StateVec& state) const -> FloatType_
    {
        const auto dx = state[StateDef_::X] - sensor_x_;
        const auto dy = state[StateDef_::Y] - sensor_y_;
        return std::sqrt(dx*dx + dy*dy);
    }
    
    auto predictBearing(const StateVec& state) const -> FloatType_
    {
        const auto dx = state[StateDef_::X] - sensor_x_;
        const auto dy = state[StateDef_::Y] - sensor_y_;
        return std::atan2(dy, dx);
    }
    
    auto predictDoppler(const StateVec& state) const -> FloatType_
    {
        const auto dx = state[StateDef_::X] - sensor_x_;
        const auto dy = state[StateDef_::Y] - sensor_y_;
        const auto vx = state[StateDef_::VX];
        const auto vy = state[StateDef_::VY];
        const auto r = std::sqrt(dx*dx + dy*dy);
        
        return (dx*vx + dy*vy) / r;
    }
    
    FloatType_ sensor_x_;
    FloatType_ sensor_y_;
};

// Position Observation Model (Updated)
template <typename FloatType_, typename StateDef_, typename CovarianceMatrixPolicy_ = math::FullCovarianceMatrixPolicy<FloatType_>>
class PositionObservationModel 
    : public ExtendedObservationModel<PositionObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                      ObservationModelTrait<FloatType_, StateDef_, 2, CovarianceMatrixPolicy_>>
{
    static_assert(motion::has_position_v<StateDef_>, 
        "StateDef must have X and Y position variables");
        
    using Base = ExtendedObservationModel<PositionObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                           ObservationModelTrait<FloatType_, StateDef_, 2, CovarianceMatrixPolicy_>>;
    
public:
    using typename Base::MeasurementVec;
    using typename Base::MeasurementCov;
    using typename Base::StateVec;
    using typename Base::JacobianMatrix;
    
    PositionObservationModel(const MeasurementVec& measurement, const MeasurementCov& covariance)
        : Base(measurement, covariance) {}
    
    auto predictMeasurement(const StateVec& state) const -> MeasurementVec override
    {
        return MeasurementVec{state[StateDef_::X], state[StateDef_::Y]};
    }
    
    auto computeJacobian(JacobianMatrix& H, const StateVec& state) const -> void override
    {
        H.setZero();
        H(0, StateDef_::X) = static_cast<FloatType_>(1.0);
        H(1, StateDef_::Y) = static_cast<FloatType_>(1.0);
    }
    
    sint32 getActiveComponentCount() const override { return 2; }
    FloatType_ getComponent(sint32 index) const override { return this->getVec()[index]; }
    FloatType_ predictComponent(sint32 index, const StateVec& state) const override
    {
        return (index == 0) ? state[StateDef_::X] : state[StateDef_::Y];
    }
    void computeJacobianRow(sint32 index, math::Vector<FloatType_, motion::state_dimension<StateDef_>>& row, 
                           const StateVec& /*state*/) const override
    {
        row.setZero();
        if (index == 0) {
            row[StateDef_::X] = 1.0;
        } else {
            row[StateDef_::Y] = 1.0;
        }
    }
    FloatType_ getComponentCovariance(sint32 index) const override { return this->getCov()(index, index); }
    bool isComponentSupported(sint32 index, const StateVec& /*state*/) const override { return true; }
};

// Range-Bearing Observation Model (Updated)
template <typename FloatType_, typename StateDef_, typename CovarianceMatrixPolicy_ = math::FullCovarianceMatrixPolicy<FloatType_>>
class RangeBearingObservationModel 
    : public ExtendedObservationModel<RangeBearingObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                      ObservationModelTrait<FloatType_, StateDef_, 2, CovarianceMatrixPolicy_>>
{
    static_assert(motion::has_position_v<StateDef_>, 
        "StateDef must have X and Y position variables");
        
    using Base = ExtendedObservationModel<RangeBearingObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                           ObservationModelTrait<FloatType_, StateDef_, 2, CovarianceMatrixPolicy_>>;
    
public:
    using typename Base::MeasurementVec;
    using typename Base::MeasurementCov;
    using typename Base::StateVec;
    using typename Base::JacobianMatrix;
    
    RangeBearingObservationModel(FloatType_ range, FloatType_ bearing, 
                                 const MeasurementCov& covariance,
                                 FloatType_ sensor_x = 0, FloatType_ sensor_y = 0)
        : Base(MeasurementVec{range, bearing}, covariance),
          sensor_x_(sensor_x), sensor_y_(sensor_y) {}
    
    auto predictMeasurement(const StateVec& state) const -> MeasurementVec override;
    auto computeJacobian(JacobianMatrix& H, const StateVec& state) const -> void override;
    
    sint32 getActiveComponentCount() const override { return 2; }
    FloatType_ getComponent(sint32 index) const override { return this->getVec()[index]; }
    FloatType_ predictComponent(sint32 index, const StateVec& state) const override;
    void computeJacobianRow(sint32 index, math::Vector<FloatType_, motion::state_dimension<StateDef_>>& row, 
                           const StateVec& state) const override;
    FloatType_ getComponentCovariance(sint32 index) const override { return this->getCov()(index, index); }
    bool isComponentSupported(sint32 index, const StateVec& /*state*/) const override { return true; }
    
private:
    FloatType_ sensor_x_;
    FloatType_ sensor_y_;
};

// Velocity Observation Model (NEW)
template <typename FloatType_, typename StateDef_, typename CovarianceMatrixPolicy_ = math::FullCovarianceMatrixPolicy<FloatType_>>
class VelocityObservationModel 
    : public ExtendedObservationModel<VelocityObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                      ObservationModelTrait<FloatType_, StateDef_, 2, CovarianceMatrixPolicy_>>
{
    static_assert(motion::has_velocity_v<StateDef_>, 
        "StateDef must have VX and VY velocity variables");
        
    using Base = ExtendedObservationModel<VelocityObservationModel<FloatType_, StateDef_, CovarianceMatrixPolicy_>, 
                                           ObservationModelTrait<FloatType_, StateDef_, 2, CovarianceMatrixPolicy_>>;
    
public:
    using typename Base::MeasurementVec;
    using typename Base::MeasurementCov;
    using typename Base::StateVec;
    using typename Base::JacobianMatrix;
    
    VelocityObservationModel(const MeasurementVec& measurement, const MeasurementCov& covariance)
        : Base(measurement, covariance) {}
    
    auto predictMeasurement(const StateVec& state) const -> MeasurementVec override
    {
        return MeasurementVec{state[StateDef_::VX], state[StateDef_::VY]};
    }
    
    auto computeJacobian(JacobianMatrix& H, const StateVec& /*state*/) const -> void override
    {
        H.setZero();
        H(0, StateDef_::VX) = static_cast<FloatType_>(1.0);
        H(1, StateDef_::VY) = static_cast<FloatType_>(1.0);
    }
    
    sint32 getActiveComponentCount() const override { return 2; }
    FloatType_ getComponent(sint32 index) const override { return this->getVec()[index]; }
    FloatType_ predictComponent(sint32 index, const StateVec& state) const override
    {
        return (index == 0) ? state[StateDef_::VX] : state[StateDef_::VY];
    }
    void computeJacobianRow(sint32 index, math::Vector<FloatType_, motion::state_dimension<StateDef_>>& row, 
                           const StateVec& /*state*/) const override
    {
        row.setZero();
        if (index == 0) {
            row[StateDef_::VX] = 1.0;
        } else {
            row[StateDef_::VY] = 1.0;
        }
    }
    FloatType_ getComponentCovariance(sint32 index) const override { return this->getCov()(index, index); }
    bool isComponentSupported(sint32 index, const StateVec& /*state*/) const override { return true; }
};
```

## Step 3: GenericUpdate Implementation with Sequential, Block, and Composed Support

```cpp
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class GenericUpdate
{
public:
    using value_type = typename CovarianceMatrixPolicy_::value_type;
    using KalmanFilterType = filter::KalmanFilter<CovarianceMatrixPolicy_>;
    using InformationFilterType = filter::InformationFilter<CovarianceMatrixPolicy_>;
    using StateVec = typename MotionModel_::StateVec;
    using CovarianceType = typename MotionModel_::CovarianceType;
    
    enum class UpdateMode {
        SEQUENTIAL,  // Apply measurements one by one (each as rank-1 update)
        BLOCK,       // Apply all measurements in a single block update
        COMPOSED,    // Combine multiple observation models into a single block update
        AUTO         // Chooses based on filter type, measurement size, and number of observation models
    };
    
    // Measurement update method with mode selection (single observation)
    template <typename ObservationModel>
    static void run(const ObservationModel& observationModel, 
                   const KalmanFilterType& filter, 
                   MotionModel_& motionModel,
                   UpdateMode mode = UpdateMode::AUTO);
    
    template <typename ObservationModel>
    static void run(const ObservationModel& observationModel, 
                   const InformationFilterType& filter, 
                   MotionModel_& motionModel,
                   UpdateMode mode = UpdateMode::AUTO);
    
    // Measurement update method with mode selection (multiple observations)
    template <typename... ObservationModels>
    static void run(const KalmanFilterType& filter, 
                   MotionModel_& motionModel,
                   const ObservationModels&... observationModels,
                   UpdateMode mode = UpdateMode::AUTO);
    
    template <typename... ObservationModels>
    static void run(const InformationFilterType& filter, 
                   MotionModel_& motionModel,
                   const ObservationModels&... observationModels,
                   UpdateMode mode = UpdateMode::AUTO);
    
    // Sequential update implementation
    template <typename ObservationModel, typename FilterType>
    static void sequentialRun(const ObservationModel& observationModel, 
                            const FilterType& filter, 
                            MotionModel_& motionModel);
    
    // Block update implementation
    template <typename ObservationModel, typename FilterType>
    static void blockRun(const ObservationModel& observationModel, 
                        const FilterType& filter, 
                        MotionModel_& motionModel);
    
    // Composed block update implementation for multiple observation models
    template <typename FilterType, typename... ObservationModels>
    static void composedRun(const FilterType& filter, 
                          MotionModel_& motionModel,
                          const ObservationModels&... observationModels);
};

// Sequential update implementation
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename ObservationModel, typename FilterType>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::sequentialRun(
    const ObservationModel& observationModel, 
    const FilterType& filter, 
    MotionModel_& motionModel)
{
    const auto activeCount = observationModel.getActiveComponentCount();
    
    for (sint32 i = 0; i < activeCount; ++i) {
        if (!observationModel.isComponentSupported(i, motionModel.getState())) {
            continue;
        }
        
        using FloatType = value_type;
        math::Vector<FloatType, 1> z_i;
        math::SquareMatrix<FloatType, 1> R_i;
        math::Matrix<FloatType, 1, MotionModel_::StateDim> H_i;
        
        z_i(0) = observationModel.getComponent(i);
        R_i(0, 0) = observationModel.getComponentCovariance(i);
        observationModel.computeJacobianRow(i, H_i.row(0), motionModel.getState());
        
        const auto h_i = observationModel.predictComponent(i, motionModel.getState());
        const auto innovation = z_i(0) - h_i;
        
        filter.updateState(motionModel.getStateRef(), motionModel.getCovarianceRef(),
                          z_i, H_i, R_i);
    }
}

// Composed block update implementation for multiple observation models
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename FilterType, typename... ObservationModels>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::composedRun(
    const FilterType& filter, 
    MotionModel_& motionModel,
    const ObservationModels&... observationModels)
{
    // Calculate total number of active measurement components
    const auto totalActiveCount = (observationModels.getActiveComponentCount() + ...);
    
    if (totalActiveCount == 0) {
        return;  // No active measurements to update
    }
    
    // Create composed measurement vector, covariance matrix, and Jacobian
    math::Vector<value_type, totalActiveCount> z;
    math::SquareMatrix<value_type, totalActiveCount> R;
    math::Matrix<value_type, totalActiveCount, MotionModel_::StateDim> H;
    
    z.setZero();
    R.setZero();
    H.setZero();
    
    sint32 rowIndex = 0;
    
    // Process each observation model
    ([&](const auto& obs) {
        const auto activeCount = obs.getActiveComponentCount();
        
        for (sint32 i = 0; i < ObservationModel::MeasurementDim; ++i) {
            if (!obs.isComponentSupported(i, motionModel.getState())) {
                continue;
            }
            
            z(rowIndex) = obs.getComponent(i);
            R(rowIndex, rowIndex) = obs.getComponentCovariance(i);
            obs.computeJacobianRow(i, H.row(rowIndex), motionModel.getState());
            
            rowIndex++;
        }
    }(observationModels), ...);
    
    // Compute predicted measurement for all active components
    math::Vector<value_type, totalActiveCount> h;
    rowIndex = 0;
    
    ([&](const auto& obs) {
        const auto activeCount = obs.getActiveComponentCount();
        
        for (sint32 i = 0; i < ObservationModel::MeasurementDim; ++i) {
            if (!obs.isComponentSupported(i, motionModel.getState())) {
                continue;
            }
            
            h(rowIndex) = obs.predictComponent(i, motionModel.getState());
            rowIndex++;
        }
    }(observationModels), ...);
    
    const auto innovation = z - h;
    
    // Perform single block update with composed matrices
    filter.updateState(motionModel.getStateRef(), motionModel.getCovarianceRef(),
                      z, H, R);
}

// Block update implementation with compile-time dispatch
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename ObservationModel, typename FilterType>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::blockRun(
    const ObservationModel& observationModel, 
    const FilterType& filter, 
    MotionModel_& motionModel)
{
    const auto activeCount = observationModel.getActiveComponentCount();
    
    // Dispatch to compile-time fixed-size implementations based on activeCount
    switch (activeCount) {
        case 1:
            blockRunImpl<1, ObservationModel, FilterType>(observationModel, filter, motionModel);
            break;
        case 2:
            blockRunImpl<2, ObservationModel, FilterType>(observationModel, filter, motionModel);
            break;
        case 3:
            blockRunImpl<3, ObservationModel, FilterType>(observationModel, filter, motionModel);
            break;
        case 4:
            blockRunImpl<4, ObservationModel, FilterType>(observationModel, filter, motionModel);
            break;
        case 5:
            blockRunImpl<5, ObservationModel, FilterType>(observationModel, filter, motionModel);
            break;
        case 6:
            blockRunImpl<6, ObservationModel, FilterType>(observationModel, filter, motionModel);
            break;
        default:
            // Fallback to sequential update for larger dimensions
            sequentialRun<ObservationModel, FilterType>(observationModel, filter, motionModel);
            break;
    }
}

// Helper implementation with compile-time fixed size
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <sint32 ActiveDim_, typename ObservationModel, typename FilterType>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::blockRunImpl(
    const ObservationModel& observationModel, 
    const FilterType& filter, 
    MotionModel_& motionModel)
{
    static_assert(ActiveDim_ > 0 && ActiveDim_ <= 6, "Active dimension must be between 1 and 6");
    
    using FloatType = value_type;
    math::Vector<FloatType, ActiveDim_> z;
    math::SquareMatrix<FloatType, ActiveDim_> R;
    math::Matrix<FloatType, ActiveDim_, MotionModel_::StateDim> H;
    
    sint32 rowIndex = 0;
    for (sint32 i = 0; i < ObservationModel::MeasurementDim; ++i) {
        if (!observationModel.isComponentSupported(i, motionModel.getState())) {
            continue;
        }
        
        z(rowIndex) = observationModel.getComponent(i);
        R(rowIndex, rowIndex) = observationModel.getComponentCovariance(i);
        observationModel.computeJacobianRow(i, H.row(rowIndex), motionModel.getState());
        
        rowIndex++;
    }
    
    math::Vector<FloatType, ActiveDim_> h;
    rowIndex = 0;
    for (sint32 i = 0; i < ObservationModel::MeasurementDim; ++i) {
        if (!observationModel.isComponentSupported(i, motionModel.getState())) {
            continue;
        }
        
        h(rowIndex) = observationModel.predictComponent(i, motionModel.getState());
        rowIndex++;
    }
    
    const auto innovation = z - h;
    
    filter.updateState(motionModel.getStateRef(), motionModel.getCovarianceRef(),
                      z, H, R);
}

// Kalman filter run method (single observation)
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename ObservationModel>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::run(
    const ObservationModel& observationModel, 
    const KalmanFilterType& filter, 
    MotionModel_& motionModel,
    UpdateMode mode)
{
    if (mode == UpdateMode::SEQUENTIAL || 
        (mode == UpdateMode::AUTO && CovarianceMatrixPolicy_::is_factored)) {
        sequentialRun<ObservationModel, KalmanFilterType>(observationModel, filter, motionModel);
    } else if (mode == UpdateMode::BLOCK || mode == UpdateMode::AUTO) {
        blockRun<ObservationModel, KalmanFilterType>(observationModel, filter, motionModel);
    }
}

// Information filter run method (single observation)
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename ObservationModel>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::run(
    const ObservationModel& observationModel, 
    const InformationFilterType& filter, 
    MotionModel_& motionModel,
    UpdateMode mode)
{
    if (mode == UpdateMode::SEQUENTIAL || 
        (mode == UpdateMode::AUTO && CovarianceMatrixPolicy_::is_factored)) {
        sequentialRun<ObservationModel, InformationFilterType>(observationModel, filter, motionModel);
    } else if (mode == UpdateMode::BLOCK || mode == UpdateMode::AUTO) {
        blockRun<ObservationModel, InformationFilterType>(observationModel, filter, motionModel);
    }
}

// Kalman filter run method (multiple observations)
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename... ObservationModels>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::run(
    const KalmanFilterType& filter, 
    MotionModel_& motionModel,
    const ObservationModels&... observationModels,
    UpdateMode mode)
{
    if (mode == UpdateMode::COMPOSED) {
        composedRun<KalmanFilterType, ObservationModels...>(filter, motionModel, observationModels...);
    } else if (mode == UpdateMode::SEQUENTIAL) {
        (sequentialRun<ObservationModels, KalmanFilterType>(observationModels, filter, motionModel), ...);
    } else if (mode == UpdateMode::BLOCK) {
        (blockRun<ObservationModels, KalmanFilterType>(observationModels, filter, motionModel), ...);
    } else {  // AUTO mode
        if (CovarianceMatrixPolicy_::is_factored) {
            (sequentialRun<ObservationModels, KalmanFilterType>(observationModels, filter, motionModel), ...);
        } else {
            composedRun<KalmanFilterType, ObservationModels...>(filter, motionModel, observationModels...);
        }
    }
}

// Information filter run method (multiple observations)
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
template <typename... ObservationModels>
void GenericUpdate<MotionModel_, CovarianceMatrixPolicy_>::run(
    const InformationFilterType& filter, 
    MotionModel_& motionModel,
    const ObservationModels&... observationModels,
    UpdateMode mode)
{
    if (mode == UpdateMode::COMPOSED) {
        composedRun<InformationFilterType, ObservationModels...>(filter, motionModel, observationModels...);
    } else if (mode == UpdateMode::SEQUENTIAL) {
        (sequentialRun<ObservationModels, InformationFilterType>(observationModels, filter, motionModel), ...);
    } else if (mode == UpdateMode::BLOCK) {
        (blockRun<ObservationModels, InformationFilterType>(observationModels, filter, motionModel), ...);
    } else {  // AUTO mode
        if (CovarianceMatrixPolicy_::is_factored) {
            (sequentialRun<ObservationModels, InformationFilterType>(observationModels, filter, motionModel), ...);
        } else {
            composedRun<InformationFilterType, ObservationModels...>(filter, motionModel, observationModels...);
        }
    }
}
```

## Step 4: Filter-Specific Measurement Updates

### Kalman Filter Measurement Update

```cpp
// In include/trackingLib/filter/kalman_filter.h
template <typename CovarianceMatrixPolicy_>
class KalmanFilter : public math::contract::CovarianceMatrixPolicyIntf<CovarianceMatrixPolicy_>
{
public:
    using value_type = typename CovarianceMatrixPolicy_::value_type;
    template <sint32 DimX_>
    using CovarianceMatrixType = typename CovarianceMatrixPolicy_::template Instantiate<DimX_>;
    
    // Add measurement update method
    template <sint32 DimX_, sint32 DimZ_>
    static void updateState(
        math::Vector<value_type, DimX_>& x,
        CovarianceMatrixType<DimX_>& P,
        const math::Vector<value_type, DimZ_>& z,
        const math::Matrix<value_type, DimZ_, DimX_>& H,
        const math::Matrix<value_type, DimZ_, DimZ_>& R);
};

// In include/trackingLib/filter/kalman_filter.hpp
template <typename CovarianceMatrixPolicy_>
template <sint32 DimX_, sint32 DimZ_>
void KalmanFilter<CovarianceMatrixPolicy_>::updateState(
    math::Vector<value_type, DimX_>& x,
    CovarianceMatrixType<DimX_>& P,
    const math::Vector<value_type, DimZ_>& z,
    const math::Matrix<value_type, DimZ_, DimX_>& H,
    const math::Matrix<value_type, DimZ_, DimZ_>& R)
{
    if constexpr (CovarianceMatrixPolicy_::is_factored)
    {
        // UDU factorization measurement update
        // Using Agee-Turner algorithm for rank-1 updates
        
        // Innovation
        const auto innovation = z - H * x;
        
        // Innovation covariance
        const auto S = H * P * H.transpose() + R;
        
        // Kalman gain in UDU form
        // K = P * H' * inv(S)
        
        // Update state
        x = x + K * innovation;
        
        // Update covariance using rank-1 updates
        // P = (I - K * H) * P
        // Applied as sequential rank-1 updates
        
        for (sint32 i = 0; i < DimZ_; ++i)
        {
            // Compute update parameters
            math::Vector<value_type, DimX_> k_col = K.column(i);
            math::Vector<value_type, DimX_> h_row = H.row(i).transpose();
            
            // Apply rank-1 update: P = P - k_col * h_row' * P
            P.rank1Update(-1.0, k_col * (h_row.transpose() * P));
        }
    }
    else
    {
        // Standard Kalman filter measurement update
        const auto S = H * P * H.transpose() + R;
        const auto K = P * H.transpose() * S.inverse();
        
        // Update state
        x = x + K * (z - H * x);
        
        // Update covariance
        P = (math::SquareMatrix<value_type, DimX_>::Identity() - K * H) * P;
        P.symmetrize();
    }
}
```

### Information Filter Measurement Update

```cpp
// In include/trackingLib/filter/information_filter.h
template <typename CovarianceMatrixPolicy_>
class InformationFilter : public math::contract::CovarianceMatrixPolicyIntf<CovarianceMatrixPolicy_>
{
public:
    using value_type = typename CovarianceMatrixPolicy_::value_type;
    template <sint32 DimX_>
    using CovarianceMatrixType = typename CovarianceMatrixPolicy_::template Instantiate<DimX_>;
    
    // Add measurement update method
    template <sint32 DimX_, sint32 DimZ_>
    static void updateState(
        math::Vector<value_type, DimX_>& y,
        CovarianceMatrixType<DimX_>& Y,
        const math::Vector<value_type, DimZ_>& z,
        const math::Matrix<value_type, DimZ_, DimX_>& H,
        const math::Matrix<value_type, DimZ_, DimZ_>& R);
};

// In include/trackingLib/filter/information_filter.hpp
template <typename CovarianceMatrixPolicy_>
template <sint32 DimX_, sint32 DimZ_>
void InformationFilter<CovarianceMatrixPolicy_>::updateState(
    math::Vector<value_type, DimX_>& y,
    CovarianceMatrixType<DimX_>& Y,
    const math::Vector<value_type, DimZ_>& z,
    const math::Matrix<value_type, DimZ_, DimX_>& H,
    const math::Matrix<value_type, DimZ_, DimZ_>& R)
{
    if constexpr (CovarianceMatrixPolicy_::is_factored)
    {
        // Information form UDU measurement update
        // Based on D'Souza and Zanetti (2019)
        
        // Information contribution
        const auto invR = R.inverse();
        const auto i = H.transpose() * invR * z;
        const auto I = H.transpose() * invR * H;
        
        // Update information vector
        y = y + i;
        
        // Update information matrix using UDU rank-1 updates
        // Y = Y + I
        // Applied as sequential rank-1 updates for each row of I
        
        for (sint32 row = 0; row < DimX_; ++row)
        {
            math::Vector<value_type, DimX_> i_row = I.row(row).transpose();
            Y.rank1Update(1.0, i_row);
        }
    }
    else
    {
        // Standard information filter measurement update
        const auto invR = R.inverse();
        const auto i = H.transpose() * invR * z;
        const auto I = H.transpose() * invR * H;
        
        // Update information vector and matrix
        y = y + i;
        Y = Y + I;
        Y.symmetrize();
    }
}
```

## Step 5: Motion Model Integration

### ExtendedMotionModel Update Method with Mode Selection

```cpp
template <typename MotionModel_, typename MotionModelTrait_>
class ExtendedMotionModel : ... 
{
    // Add update method to ExtendedMotionModel with mode selection (single observation)
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, 
               const KalmanFilterType& filter,
               generic::UpdateMode mode = generic::UpdateMode::AUTO)
    {
        generic::Update<MotionModel_, CovarianceMatrixPolicy_>::run(
            observationModel, filter, static_cast<MotionModel_&>(*this), mode);
    }
    
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, 
               const InformationFilterType& filter,
               generic::UpdateMode mode = generic::UpdateMode::AUTO)
    {
        generic::Update<MotionModel_, CovarianceMatrixPolicy_>::run(
            observationModel, filter, static_cast<MotionModel_&>(*this), mode);
    }
    
    // Add update method to ExtendedMotionModel with mode selection (multiple observations)
    template <typename... ObservationModels>
    void update(const KalmanFilterType& filter,
               const ObservationModels&... observationModels,
               generic::UpdateMode mode = generic::UpdateMode::AUTO)
    {
        generic::Update<MotionModel_, CovarianceMatrixPolicy_>::run(
            filter, static_cast<MotionModel_&>(*this), observationModels..., mode);
    }
    
    template <typename... ObservationModels>
    void update(const InformationFilterType& filter,
               const ObservationModels&... observationModels,
               generic::UpdateMode mode = generic::UpdateMode::AUTO)
    {
        generic::Update<MotionModel_, CovarianceMatrixPolicy_>::run(
            filter, static_cast<MotionModel_&>(*this), observationModels..., mode);
    }
    
    // Helper methods for state and covariance access
    auto getState() const -> const StateVec& { return state_; }
    auto getStateRef() -> StateVec& { return state_; }
    auto getCovariance() const -> const CovarianceType& { return covariance_; }
    auto getCovarianceRef() -> CovarianceType& { return covariance_; }
};
```

### IMotionModel Interface Extension

```cpp
template <typename CovarianceMatrixPolicy_>
class IMotionModel : public ... 
{
    // ... existing methods ...
    
    // Add update methods to interface with mode selection (single observation)
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, 
               const KalmanFilterType& filter,
               generic::UpdateMode mode = generic::UpdateMode::AUTO) = 0;
    
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, 
               const InformationFilterType& filter,
               generic::UpdateMode mode = generic::UpdateMode::AUTO) = 0;
    
    // Add update methods to interface with mode selection (multiple observations)
    template <typename... ObservationModels>
    void update(const KalmanFilterType& filter,
               const ObservationModels&... observationModels,
               generic::UpdateMode mode = generic::UpdateMode::AUTO) = 0;
    
    template <typename... ObservationModels>
    void update(const InformationFilterType& filter,
               const ObservationModels&... observationModels,
               generic::UpdateMode mode = generic::UpdateMode::AUTO) = 0;
    
    // Helper methods for state and covariance access
    virtual auto getState() const -> const StateVec& = 0;
    virtual auto getStateRef() -> StateVec& = 0;
    virtual auto getCovariance() const -> const CovarianceType& = 0;
    virtual auto getCovarianceRef() -> CovarianceType& = 0;
};
```

### C++17 Features in Observation Model Framework

#### 1. `[[nodiscard]]` Attribute
- Applied to all getter methods to prevent unused return values
- Improves code safety by catching potential bugs at compile-time
- Example: `[[nodiscard]] virtual auto getMeasurement() const noexcept -> const MeasurementVec& = 0;`

#### 2. `noexcept` Specifications
- All methods marked as `noexcept` to indicate they won't throw exceptions
- Improves code generation and allows for more aggressive compiler optimization
- Important for real-time systems and safety-critical applications

#### 3. Default Member Initializers
- Modern C++17 syntax for default initialization of member variables
- Example: `FloatType_ sensor_x_{0.0};` instead of initializing in constructor

#### 4. Structured Bindings (for Usage)
- Allow cleaner extraction of components from measurement vectors
- Example:
  ```cpp
  auto [range, bearing] = obs.predictMeasurement(state);
  ```

#### 5. Inline Variables
- In the state trait system for cleaner compile-time constants
- Example: `inline constexpr bool has_position_v = detail::has_position_impl<StateDef>::value;`

#### 6. if constexpr for SFINAE
- Can be used in observation model implementations for compile-time condition checks
- Example in RangeBearingDoppler model:
  ```cpp
  auto computeJacobian(JacobianMatrix& H, const StateVec& state) const noexcept -> void override
  {
      // Position derivatives always present
      const auto dx = state[StateDef_::X] - sensor_x_;
      const auto dy = state[StateDef_::Y] - sensor_y_;
      const auto r2 = dx * dx + dy * dy;
      const auto r = std::sqrt(r2);
      
      H.setZero();
      H(0, StateDef_::X) = dx / r;
      H(0, StateDef_::Y) = dy / r;
      H(1, StateDef_::X) = -dy / r2;
      H(1, StateDef_::Y) = dx / r2;
      
      // Velocity derivatives only if StateDef has velocity
      if constexpr (motion::has_velocity_v<StateDef_>) {
          const auto vx = state[StateDef_::VX];
          const auto vy = state[StateDef_::VY];
          const auto dv = dx*vx + dy*vy;
          
          H(0, StateDef_::VX) = vx / r - (dx * dv) / (r * r2);
          H(0, StateDef_::VY) = vy / r - (dy * dv) / (r * r2);
          H(1, StateDef_::VX) = (vx * dy - vy * dx) / r2;
          H(1, StateDef_::VY) = (vy * dx - vx * dy) / r2;
          H(2, StateDef_::VX) = (vx * dx + vy * dy) / r2;
          H(2, StateDef_::VY) = (vx * dy + vy * dx) / r2;
      }
  }
  ```

#### 7. Fold Expressions (for Future Extensions)
- Can simplify component-wise operations in observation models
- Example for combining multiple observation components:
  ```cpp
  template <typename... ObservationModels>
  auto combineObservations(const ObservationModels&... obs)
  {
      // Folds to combine measurements from multiple observation models
      auto measurements = (obs.getMeasurement(), ...);
  }
  ```

### Benefits of C++17 Improvements

| Feature | Benefit |
|---------|---------|
| `[[nodiscard]]` | Prevents unused return values, catching bugs early |
| `noexcept` | Enables aggressive optimization, better real-time performance |
| Default Member Initializers | Cleaner code, reduces constructor verbosity |
| Structured Bindings | More readable code when working with measurements |
| Inline Variables | Simplifies trait system implementation |
| `if constexpr` | Compile-time conditional logic with zero runtime overhead |
| Fold Expressions | Enables elegant variadic template patterns for future extensions |

### Compatibility Considerations

- All changes maintain backward compatibility with existing StateDef structures (StateDefCV, StateDefCA)
- Code compiles with C++17 and later standards
- Older compilers may require upgrades (GCC 7+, Clang 5+, MSVC 2017+)
- No breaking changes to existing API

### Usage Example with RangeBearingDoppler Observation (Composed Update)

```cpp
// Example: Tracking with RADAR measurements (Range-Bearing-Doppler)
using namespace tracking;

// Define types
using FloatType = float32;
using CovPolicy = math::FullCovarianceMatrixPolicy<FloatType>;
using MotionModel = motion::MotionModelCV<CovPolicy>;
using KalmanFilterType = filter::KalmanFilter<CovPolicy>;

// Create motion model
auto state = MotionModel::StateVecFromList({10.0f, 1.0f, 5.0f, 0.5f});
auto cov = MotionModel::StateCovFromList({{1.0f, 0.0f, 0.0f, 0.0f},
                                           {0.0f, 0.1f, 0.0f, 0.0f},
                                           {0.0f, 0.0f, 1.0f, 0.0f},
                                           {0.0f, 0.0f, 0.0f, 0.1f}});
MotionModel cvModel(state, cov);

// Create Range-Bearing-Doppler observation model using trait system
using RadarObs = observation::RangeBearingDopplerObservationModel<FloatType, MotionModel::StateDef, CovPolicy>;
RadarObs::MeasurementVec z{10.5f, 0.48f, 0.9f};  // range (m), bearing (radians), doppler (m/s)
RadarObs::MeasurementCov R = math::SquareMatrix<FloatType, 3>::Diagonal(
    {0.1f, 0.01f, 0.05f});  // range, bearing, doppler variances
RadarObs radarObs(z, R, 0, 0);  // sensor at origin

// Create additional observations (e.g., from LiDAR and camera)
using LiDARObs = observation::PositionObservationModel<FloatType, MotionModel::StateDef, CovPolicy>;
LiDARObs::MeasurementVec zLidar{10.3f, 5.1f};
LiDARObs::MeasurementCov RLidar = math::SquareMatrix<FloatType, 2>::Diagonal({0.05f, 0.05f});
LiDARObs lidarObs(zLidar, RLidar);

using CameraObs = observation::RangeBearingObservationModel<FloatType, MotionModel::StateDef, CovPolicy>;
CameraObs::MeasurementVec zCamera{10.6f, 0.47f};
CameraObs::MeasurementCov RCamera = math::SquareMatrix<FloatType, 2>::Diagonal({0.2f, 0.02f});
CameraObs cameraObs(zCamera, RCamera, 0.5f, 0.5f);  // camera offset

// Create Kalman filter
KalmanFilterType filter;

// Option 1: Automatic mode selection (single observation)
cvModel.update(radarObs, filter);

// Reset model to initial state
cvModel.setState(state);
cvModel.setCovariance(cov);

// Option 2: Composed update with multiple observations (single block update)
cvModel.update(filter, radarObs, lidarObs, cameraObs, 
               generic::UpdateMode::COMPOSED);

// Verify the update worked
std::cout << "Updated state: " << cvModel.getVec() << std::endl;
std::cout << "Updated covariance: " << cvModel.getCov() << std::endl;

// Option 3: Sequential update for comparison
MotionModel cvModelSequential(state, cov);
cvModelSequential.update(filter, radarObs, lidarObs, cameraObs, 
                       generic::UpdateMode::SEQUENTIAL);

// Results should be identical (within numerical tolerance)
EXPECT_TRUE(cvModel.getState().isApprox(cvModelSequential.getState(), 1e-6));
EXPECT_TRUE(cvModel.getCovariance().isApprox(cvModelSequential.getCovariance(), 1e-6));
```

## Step 6: Testing Infrastructure

### Test Files to Create/Update

- `tests/observation/test_iobservation_model.cpp` - Base interface tests
- `tests/observation/test_position_observation_model.cpp` - Position observation tests
- `tests/observation/test_velocity_observation_model.cpp` - Velocity observation tests
- `tests/observation/test_range_bearing_observation_model.cpp` - Range-bearing tests
- `tests/observation/test_range_bearing_doppler_observation_model.cpp` - Range-bearing-Doppler tests
- `tests/motion/test_measurement_update.cpp` - Motion model integration tests
- `tests/motion/test_composed_update.cpp` - Composed multi-observation update tests
- `tests/filter/test_measurement_update.cpp` - Filter-specific update tests
- `tests/filter/test_sequential_update.cpp` - Sequential update tests
- `tests/filter/test_block_update.cpp` - Block update tests
- `tests/filter/test_composed_update.cpp` - Composed update tests
- `tests/filter/test_update_mode_selection.cpp` - Auto mode selection tests
- Updates to `tests/motion/test_motion_model_cv.cpp`
- Updates to `tests/motion/test_motion_model_ca.cpp`

### Test Coverage

#### Core Functionality Tests
- Kalman filter measurement update (full and factored)
- Information filter measurement update (full and factored)
- Observation model Jacobians for all types
- RangeBearingDoppler with velocity derivatives
- Sequential update vs block update consistency
- Motion model compatibility checks
- State trait system verification
- Active component detection
- Unsupported state variable handling
- Integration with motion models (CV, CA, position-only)
- Numerical stability and edge cases
- Update mode selection (auto, sequential, block)

#### RangeBearingDoppler Specific Tests
```cpp
// Test 1: RangeBearingDoppler with CV model
TEST(RangeBearingDopplerObservation, WithCVModel)
{
    using FloatType = float32;
    using StateDef = motion::StateDefCV;
    using ObsModel = RangeBearingDopplerObservationModel<FloatType, StateDef>;
    
    ObsModel obs(10.0f, 0.0f, 1.0f, ...);
    EXPECT_EQ(obs.getActiveComponentCount(), 3);
    
    // Test predictMeasurement and computeJacobian
    StateVec state{10.0f, 1.0f, 0.0f, 0.0f};
    auto predicted = obs.predictMeasurement(state);
    EXPECT_NEAR(predicted(0), 10.0f, 1e-6);
    EXPECT_NEAR(predicted(1), 0.0f, 1e-6);
    EXPECT_NEAR(predicted(2), 1.0f, 1e-6);
}

// Test 2: RangeBearingDoppler with position-only state
TEST(RangeBearingDopplerObservation, WithPositionOnlyModel)
{
    struct StateDefPositionOnly {
        enum _ { X = 0, Y, NUM_STATE_VARIABLES };
    };
    
    using ObsModel = RangeBearingDopplerObservationModel<float32, StateDefPositionOnly>;
    ObsModel obs(10.0f, 0.0f, 1.0f, ...);
    
    EXPECT_EQ(obs.getActiveComponentCount(), 2);  // only range and bearing
}

// Test 3: Sequential vs block update consistency
TEST(MeasurementUpdate, SequentialVsBlock)
{
    // Create identical motion model and observation
    MotionModel model1 = createTestModel();
    MotionModel model2 = createTestModel();
    ObservationModel obs = createTestObservation();
    
    // Apply sequential update
    sequentialUpdate(filter, model1, obs);
    
    // Apply block update
    blockUpdate(filter, model2, obs);
    
    // Should produce identical results
    EXPECT_TRUE(model1.getState().isApprox(model2.getState(), 1e-6));
    EXPECT_TRUE(model1.getCovariance().isApprox(model2.getCovariance(), 1e-6));
}

// Test 4: Composed update with multiple observation models
TEST(MeasurementUpdate, ComposedUpdate)
{
    // Create identical motion models
    MotionModel modelComposed = createTestModel();
    MotionModel modelSequential = createTestModel();
    
    // Create multiple observation models
    auto obs1 = createTestObservation1();
    auto obs2 = createTestObservation2();
    auto obs3 = createTestObservation3();
    
    // Apply composed block update
    composedUpdate(filter, modelComposed, obs1, obs2, obs3);
    
    // Apply sequential updates
    sequentialUpdate(filter, modelSequential, obs1);
    sequentialUpdate(filter, modelSequential, obs2);
    sequentialUpdate(filter, modelSequential, obs3);
    
    // Should produce identical results
    EXPECT_TRUE(modelComposed.getState().isApprox(modelSequential.getState(), 1e-6));
    EXPECT_TRUE(modelComposed.getCovariance().isApprox(modelSequential.getCovariance(), 1e-6));
}

// Test 5: Composed update with different observation types
TEST(MeasurementUpdate, ComposedUpdateDifferentTypes)
{
    using FloatType = float32;
    using StateDef = motion::StateDefCV;
    using CovPolicy = math::FullCovarianceMatrixPolicy<FloatType>;
    using MotionModel = motion::MotionModelCV<CovPolicy>;
    using KalmanFilterType = filter::KalmanFilter<CovPolicy>;
    
    MotionModel model1 = createTestModel();
    MotionModel model2 = createTestModel();
    
    // Create different observation types
    using RadarObs = observation::RangeBearingDopplerObservationModel<FloatType, StateDef, CovPolicy>;
    RadarObs radarObs(10.5f, 0.48f, 0.9f, 
                     math::SquareMatrix<FloatType, 3>::Diagonal({0.1f, 0.01f, 0.05f}), 0, 0);
    
    using LiDARObs = observation::PositionObservationModel<FloatType, StateDef, CovPolicy>;
    LiDARObs lidarObs({10.3f, 5.1f}, 
                     math::SquareMatrix<FloatType, 2>::Diagonal({0.05f, 0.05f}));
    
    using CameraObs = observation::RangeBearingObservationModel<FloatType, StateDef, CovPolicy>;
    CameraObs cameraObs(10.6f, 0.47f, 
                     math::SquareMatrix<FloatType, 2>::Diagonal({0.2f, 0.02f}), 0.5f, 0.5f);
    
    KalmanFilterType filter;
    
    // Apply composed block update
    composedUpdate(filter, model1, radarObs, lidarObs, cameraObs);
    
    // Apply sequential updates
    sequentialUpdate(filter, model2, radarObs);
    sequentialUpdate(filter, model2, lidarObs);
    sequentialUpdate(filter, model2, cameraObs);
    
    // Should produce identical results
    EXPECT_TRUE(model1.getState().isApprox(model2.getState(), 1e-6));
    EXPECT_TRUE(model1.getCovariance().isApprox(model2.getCovariance(), 1e-6));
}

// Test 6: Empty composed update (no active measurements)
TEST(MeasurementUpdate, ComposedUpdateNoMeasurements)
{
    MotionModel model = createTestModel();
    auto initialState = model.getState();
    auto initialCovariance = model.getCovariance();
    
    // Create observation model with no active components (position-only state for velocity observation)
    struct StateDefPositionOnly {
        enum _ { X = 0, Y, NUM_STATE_VARIABLES };
    };
    
    using VelocityObs = observation::VelocityObservationModel<float32, StateDefPositionOnly>;
    VelocityObs obs({1.0f, 0.5f}, {{0.1f, 0.0f}, {0.0f, 0.1f}});
    
    // Should not change state
    composedUpdate(filter, model, obs);
    
    EXPECT_TRUE(model.getState().isApprox(initialState, 1e-10));
    EXPECT_TRUE(model.getCovariance().isApprox(initialCovariance, 1e-10));
}
```

#### C++17 Feature Tests
```cpp
// Test 4: [[nodiscard]] attribute enforcement
TEST(ObservationModelCxx17, NoDiscardAttribute)
{
    // This should generate a compiler warning if return value is unused
    PositionObservationModel<float32, StateDefCV, FullCovarianceMatrixPolicy<float32>> obs(
        {0.0f, 0.0f}, {{1.0f, 0.0f}, {0.0f, 1.0f}});
    obs.getMeasurement();  // Should warn about unused result
}

// Test 5: State trait system with C++17 features
TEST(StateDefTraitsCxx17, CompileTimeConstants)
{
    static_assert(motion::has_position_v<StateDefCV>, "StateDefCV must have position");
    static_assert(motion::has_velocity_v<StateDefCV>, "StateDefCV must have velocity");
    static_assert(!motion::has_acceleration_v<StateDefCV>, "StateDefCV should not have acceleration");
    static_assert(motion::state_dimension<StateDefCV> == 4, "StateDefCV must be 4-dimensional");
}

// Test 6: if constexpr velocity check
TEST(ObservationModelCxx17, IfConstexprVelocityCheck)
{
    // For StateDefCV (has velocity)
    EXPECT_TRUE(doesStateHaveVelocity<StateDefCV>());
    
    // For StateDefPositionOnly (no velocity)
    EXPECT_FALSE(doesStateHaveVelocity<StateDefPositionOnly>());
}
```

## Step 7: Multi-Object Tracking Test Data Generation

### Test Data Generation Strategy

To demonstrate trackinglib's multi-object tracking capabilities, we'll use Python-based simulation for test data generation. This approach is lightweight, cross-platform, and easy to integrate with existing tests.

### Recommended Open-Source Tools for Realistic Test Data

#### 1. **TrackingLab (Python) - Simple but Realistic**
- **What it is**: Lightweight Python simulation framework specifically for multi-object tracking
- **Key Features**:
  - Simulates realistic motion models (constant velocity, constant acceleration, coordinated turn)
  - Generates range-bearing, range-bearing-doppler, and Cartesian measurements
  - Supports sensor models (RADAR, LiDAR, camera)
  - Includes clutter and missed detection models
  - Outputs CSV/JSON data files with ground truth and measurements
- **Repository**: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python (Chapter 13 has tracking examples)
- **Integration**: Write Python scripts to generate data and save as CSV, then load in C++ tests

#### 2. **PyFilter (Professional Grade)**
- **What it is**: Python framework for state estimation and tracking
- **Key Features**:
  - Multi-object tracking with PHD/CPHD filters
  - Simulates various motion models including realistic maneuvering
  - Sensor simulation (RADAR, LiDAR) with realistic noise models
  - Clutter and detection probability modeling
  - Visualization support (Matplotlib)
- **Repository**: https://github.com/rlabbe/PyFilter

#### 3. **MATLAB/Simulink Tracking Toolbox (Commercial, but Octave Compatible)**
- **What it is**: Comprehensive tracking simulation framework
- **Key Features**:
  - Multi-object tracking simulation with realistic motion models
  - Various motion models (CV, CA, CT, maneuvering)
  - Sensor models (RADAR, LiDAR, camera, sonar) with realistic characteristics
  - Detection and clutter models based on real sensor data
  - Visualization tools and metric calculation
- **Alternatives**: Octave/Octave-Forge (free MATLAB clone) with tracking toolbox functionality

#### 4. **CARLA Simulator (High-Fidelity Autonomous Driving)**
- **What it is**: Open-source autonomous driving simulator
- **Key Features**:
  - Photorealistic 3D environment
  - Realistic vehicle dynamics with physics simulation
  - Sensor simulation (LiDAR, RADAR, cameras, GPS) with real-world characteristics
  - Multi-vehicle simulation with traffic scenarios
  - ROS interface for integration with other tools
- **Repository**: https://github.com/carla-simulator/carla

#### 5. **Gazebo (Robotics Simulator)**
- **What it is**: 3D robotics simulator with physics engine
- **Key Features**:
  - Physics-based simulation with realistic dynamics
  - Sensor models (LiDAR, RADAR, cameras) with noise and occlusion
  - Multi-robot/target simulation with collision detection
  - ROS integration for easy data collection
- **Repository**: https://gazebosim.org/

#### 6. **OpenSceneGraph (3D Visualization and Simulation)**
- **What it is**: High-performance 3D graphics toolkit
- **Key Features**:
  - 3D visualization of tracking scenarios
  - Physics simulation for realistic object motion
  - Sensor simulation for various modalities
  - Cross-platform support
- **Repository**: https://github.com/openscenegraph/OpenSceneGraph

### Improved Python Data Generation Script with Realistic Motion

```python
#!/usr/bin/env python3
"""
Tool to generate realistic multi-object tracking test data with maneuvering.
Uses realistic motion models including coordinated turns and maneuvering.
"""

import numpy as np
import csv
import argparse
import matplotlib.pyplot as plt

def simulate_maneuvering_vehicle(num_steps, dt, initial_state):
    """Simulate a vehicle with possible maneuvers (coordinated turns)."""
    states = np.zeros((num_steps, 4))  # [x, y, vx, vy]
    states[0] = initial_state
    
    # Possible maneuvers: 0=straight, 1=left turn, 2=right turn
    maneuver_prob = 0.02  # 2% chance per step to start maneuver
    current_maneuver = 0
    turn_rate = 0.1  # radians per second
    
    for t in range(1, num_steps):
        if np.random.rand() < maneuver_prob:
            current_maneuver = np.random.choice([0, 1, 2])
        
        x, y, vx, vy = states[t-1]
        speed = np.sqrt(vx**2 + vy**2)
        
        if current_maneuver == 0:
            # Straight line
            states[t] = [x + vx*dt, y + vy*dt, vx, vy]
        else:
            # Coordinated turn
            turn_dir = 1 if current_maneuver == 1 else -1
            omega = turn_dir * turn_rate
            
            # Compute new velocity
            new_vx = speed * np.cos(omega*dt)
            new_vy = speed * np.sin(omega*dt)
            
            # Compute new position
            if omega != 0:
                new_x = x + (speed/omega)*(np.sin(omega*dt))
                new_y = y + (speed/omega)*(1 - np.cos(omega*dt))
            else:
                new_x = x + vx*dt
                new_y = y + vy*dt
            
            states[t] = [new_x, new_y, new_vx, new_vy]
    
    return states

def simulate_multi_object_tracking(num_objects, num_steps, dt, seed=42):
    np.random.seed(seed)
    
    # Initialize object states with realistic initial conditions
    states = np.random.randn(num_objects, 4) * np.array([100, 100, 5, 5])
    states[:, 2:4] = np.clip(states[:, 2:4], -10, 10)  # Limit initial velocity
    
    ground_truth = np.zeros((num_steps, num_objects, 4))
    
    for obj in range(num_objects):
        ground_truth[:, obj] = simulate_maneuvering_vehicle(
            num_steps, dt, states[obj]
        )
    
    measurements = []
    for t in range(num_steps):
        t_measurements = []
        for obj in range(num_objects):
            x, y, vx, vy = ground_truth[t, obj]
            
            # Range-bearing-doppler measurements with realistic noise
            range_ = np.sqrt(x**2 + y**2) + np.random.normal(0, 1.5)
            bearing = np.arctan2(y, x) + np.random.normal(0, 0.02)
            doppler = (x*vx + y*vy) / np.sqrt(x**2 + y**2) + np.random.normal(0, 0.8)
            
            t_measurements.append([range_, bearing, doppler])
        
        measurements.append(np.array(t_measurements))
    
    return ground_truth, np.array(measurements)

def save_to_csv(ground_truth, measurements, dt, filename_prefix):
    """Save ground truth and measurements to CSV files."""
    
    # Ground truth file
    gt_filename = f"{filename_prefix}_ground_truth.csv"
    with open(gt_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time", "object_id", "x", "y", "vx", "vy"])
        
        for t in range(ground_truth.shape[0]):
            for obj in range(ground_truth.shape[1]):
                time = t * dt
                x, y, vx, vy = ground_truth[t, obj]
                writer.writerow([time, obj, x, y, vx, vy])
    
    # Measurements file
    meas_filename = f"{filename_prefix}_measurements.csv"
    with open(meas_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time", "object_id", "range", "bearing", "doppler"])
        
        for t in range(measurements.shape[0]):
            for obj in range(measurements.shape[1]):
                time = t * dt
                range_, bearing, doppler = measurements[t, obj]
                writer.writerow([time, obj, range_, bearing, doppler])
    
    print(f"Data saved to {gt_filename} and {meas_filename}")

def plot_scenario(ground_truth, measurements, dt):
    """Plot ground truth trajectories and measurements."""
    num_objects = ground_truth.shape[1]
    num_steps = ground_truth.shape[0]
    
    plt.figure(figsize=(12, 6))
    
    # Plot ground truth
    for obj in range(num_objects):
        plt.plot(ground_truth[:, obj, 0], ground_truth[:, obj, 2], 
                label=f"GT Object {obj}", linewidth=2)
    
    # Plot measurements
    for obj in range(num_objects):
        x_meas = []
        y_meas = []
        for t in range(num_steps):
            range_, bearing, _ = measurements[t, obj]
            x = range_ * np.cos(bearing)
            y = range_ * np.sin(bearing)
            x_meas.append(x)
            y_meas.append(y)
        
        plt.scatter(x_meas, y_meas, 
                   label=f"Meas Object {obj}", s=10, alpha=0.7)
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Multi-Object Tracking Scenario')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('multiobject_scenario.png', dpi=300)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Generate multi-object tracking test data")
    parser.add_argument("--objects", type=int, default=5, help="Number of objects to track")
    parser.add_argument("--steps", type=int, default=100, help="Number of time steps")
    parser.add_argument("--dt", type=float, default=0.1, help="Time step in seconds")
    parser.add_argument("--prefix", type=str, default="test_data", help="Output filename prefix")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    parser.add_argument("--plot", action="store_true", help="Generate scenario plot")
    
    args = parser.parse_args()
    
    ground_truth, measurements = simulate_multi_object_tracking(
        args.objects, args.steps, args.dt, args.seed
    )
    
    save_to_csv(ground_truth, measurements, args.dt, args.prefix)
    
    if args.plot:
        plot_scenario(ground_truth, measurements, args.dt)

if __name__ == "__main__":
    main()
```

### Using Real-World Datasets

For the most realistic test data, consider using existing real-world multi-object tracking datasets:

#### 1. **MOT Challenge Datasets**
- **What it is**: Standard benchmark datasets for multi-object tracking
- **Key Features**:
  - Real-world pedestrian tracking in various scenarios (urban, indoor, crowded)
  - Multiple camera views with synchronized data
  - Ground truth annotations with IDs and bounding boxes
  - Metrics computation (MOTA, MOTP, etc.)
- **Available Datasets**:
  - MOT17: 14 sequences with 2D bounding boxes
  - MOT20: 8 sequences with 2D bounding boxes
  - MOT16: Previous version with 11 sequences
- **Download**: https://motchallenge.net/
- **Format Conversion**: Convert bounding boxes to appropriate measurement format (e.g., center position for trackinglib)

#### 2. **KITTI Tracking Dataset**
- **What it is**: Autonomous driving tracking dataset
- **Key Features**:
  - 21 training sequences, 29 test sequences
  - Vehicle and pedestrian tracking
  - LiDAR and camera data
  - Velocity and 3D bounding box annotations
- **Download**: http://www.cvlibs.net/datasets/kitti/eval_tracking.php
- **Format Conversion**: Extract range-bearing-doppler measurements from LiDAR data

#### 3. **nuScenes Dataset**
- **What it is**: Large-scale autonomous driving dataset
- **Key Features**:
  - 1000 scenes with 20 seconds each
  - 6 cameras, LiDAR, RADAR, IMU, GPS
  - 3D bounding boxes with semantic labels
  - Velocity and acceleration annotations
- **Download**: https://www.nuscenes.org/
- **Format Conversion**: Extract sensor data for trackinglib tests

#### 4. **Waymo Open Dataset**
- **What it is**: Autonomous driving dataset from Waymo
- **Key Features**:
  - LiDAR and camera data
  - Vehicle, pedestrian, cyclist annotations
  - Velocity and acceleration information
- **Download**: https://waymo.com/open/

### Usage Instructions

#### For Generated Data:
1. Save the script as `generate_multiobject_data.py` in the `tools/` directory
2. Install dependencies: `pip install numpy matplotlib argparse`
3. Generate test data:
   ```bash
   python generate_multiobject_data.py --objects 5 --steps 100 --dt 0.1 --prefix test_data --plot
   ```

#### For Real-World Data:
1. Download a dataset (e.g., MOT17 from https://motchallenge.net/)
2. Use the provided tools to extract tracking data
3. Convert to the CSV format used by trackinglib tests
4. Place in `tests/data/` directory
5. Run tests using the real-world data

### Test Data Format

#### Ground Truth CSV
```csv
time,object_id,x,y,vx,vy
0.0,0,100.0,50.0,5.0,2.0
0.0,1,-20.0,30.0,-3.0,4.0
0.1,0,100.5,50.2,5.0,2.0
0.1,1,-20.3,30.4,-3.0,4.0
```

#### Measurements CSV
```csv
time,object_id,range,bearing,doppler
0.0,0,111.80,0.463,5.89
0.0,1,36.06,2.158,-1.11
0.1,0,112.35,0.461,5.91
0.1,1,36.54,2.162,-1.09
```

### C++ Test Integration

```cpp
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "trackingLib/motion/motion_model_cv.h"
#include "trackingLib/observation/range_bearing_doppler_observation_model.h"

struct GroundTruthState {
    float64 time;
    sint32 objectId;
    float64 x, y, vx, vy;
};

struct Measurement {
    float64 time;
    sint32 objectId;
    float64 range, bearing, doppler;
};

std::vector<GroundTruthState> loadGroundTruth(const std::string& filename) {
    std::vector<GroundTruthState> states;
    std::ifstream file(filename);
    std::string line;
    
    if (file.is_open()) {
        std::getline(file, line); // Skip header
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            GroundTruthState state;
            char separator;
            
            ss >> state.time >> separator >> state.objectId >> separator
               >> state.x >> separator >> state.y >> separator
               >> state.vx >> separator >> state.vy;
            
            states.push_back(state);
        }
    }
    
    return states;
}

std::vector<Measurement> loadMeasurements(const std::string& filename) {
    std::vector<Measurement> measurements;
    std::ifstream file(filename);
    std::string line;
    
    if (file.is_open()) {
        std::getline(file, line); // Skip header
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            Measurement m;
            char separator;
            
            ss >> m.time >> separator >> m.objectId >> separator
               >> m.range >> separator >> m.bearing >> separator >> m.doppler;
            
            measurements.push_back(m);
        }
    }
    
    return measurements;
}

TEST(MultiObjectTrackingTest, RangeBearingDopplerTracking) {
    using FloatType = float64;
    using CovPolicy = math::FullCovarianceMatrixPolicy<FloatType>;
    using MotionModel = motion::MotionModelCV<CovPolicy>;
    using KalmanFilterType = filter::KalmanFilter<CovPolicy>;
    using RadarObs = observation::RangeBearingDopplerObservationModel<FloatType, MotionModel::StateDef>;
    
    auto groundTruth = loadGroundTruth("data/ground_truth.csv");
    auto measurements = loadMeasurements("data/measurements.csv");
    
    std::map<float64, std::map<sint32, Measurement>> groupedMeasurements;
    for (const auto& m : measurements) {
        groupedMeasurements[m.time][m.objectId] = m;
    }
    
    std::map<sint32, MotionModel> trackers;
    KalmanFilterType filter;
    
    for (const auto& timeStep : groupedMeasurements) {
        const auto& time = timeStep.first;
        const auto& objectMeasurements = timeStep.second;
        
        for (const auto& [objectId, measurement] : objectMeasurements) {
            if (trackers.find(objectId) == trackers.end()) {
                const auto x = measurement.range * std::cos(measurement.bearing);
                const auto y = measurement.range * std::sin(measurement.bearing);
                const auto vx = measurement.doppler * std::cos(measurement.bearing);
                const auto vy = measurement.doppler * std::sin(measurement.bearing);
                
                auto initialState = MotionModel::StateVecFromList({x, vx, y, vy});
                auto initialCov = MotionModel::StateCovFromList({
                    {100.0, 0.0, 0.0, 0.0},
                    {0.0, 25.0, 0.0, 0.0},
                    {0.0, 0.0, 100.0, 0.0},
                    {0.0, 0.0, 0.0, 25.0}
                });
                
                trackers[objectId] = MotionModel(initialState, initialCov);
            }
            
            RadarObs::MeasurementVec z{measurement.range, measurement.bearing, measurement.doppler};
            RadarObs::MeasurementCov R = math::SquareMatrix<FloatType, 3>::Diagonal({1.0, 0.01, 0.25});
            RadarObs obs(z, R);
            
            trackers[objectId].update(obs, filter);
        }
    }
    
    for (const auto& [objectId, tracker] : trackers) {
        auto lastGT = std::find_if(groundTruth.rbegin(), groundTruth.rend(),
                                  [objectId](const GroundTruthState& s) {
                                      return s.objectId == objectId;
                                  });
        
        if (lastGT != groundTruth.rend()) {
            const auto& estimatedState = tracker.getState();
            const auto& trueState = *lastGT;
            
            EXPECT_NEAR(estimatedState[0], trueState.x, 10.0);
            EXPECT_NEAR(estimatedState[2], trueState.y, 10.0);
            EXPECT_NEAR(estimatedState[1], trueState.vx, 5.0);
            EXPECT_NEAR(estimatedState[3], trueState.vy, 5.0);
        }
    }
}
```

### Implementation Files to Create/Update

```
include/trackingLib/
├── motion/
│   ├── state_def_traits.h          # NEW: Type traits for StateDef (C++17)
│   └── ...
└── observation/                     # NEW: Observation model directory
    ├── iobservation_model.h         # NEW: Pure abstract interface (C++17)
    ├── extended_observation_model.h # NEW: Partial implementation with StateMem inheritance (C++17)
    ├── observation_model_traits.h   # NEW: Helper traits for observation models (C++17)
    ├── range_bearing_doppler_observation_model.h  # NEW: Range-Bearing-Doppler model
    ├── range_bearing_observation_model.h         # NEW: Range-Bearing model
    ├── position_observation_model.h              # NEW: Position model
    └── velocity_observation_model.h              # NEW: Velocity model
```

### Test File Organization

```
tests/
├── observation/
│   ├── test_iobservation_model.cpp           # Pure interface tests
│   ├── test_extended_observation_model.cpp   # Partial implementation tests
│   ├── test_observation_model_traits.cpp     # Trait system tests
│   ├── test_position_observation_model.cpp   # Position observation tests
│   ├── test_velocity_observation_model.cpp   # Velocity observation tests
│   ├── test_range_bearing_observation_model.cpp # Range-bearing tests
│   └── test_range_bearing_doppler_observation_model.cpp # Range-bearing-Doppler tests
├── motion/
│   ├── test_measurement_update.cpp           # Motion model integration tests
│   ├── test_motion_model_cv.cpp              # CV model with observation update
│   ├── test_motion_model_ca.cpp              # CA model with observation update
│   └── test_multiobject_tracking.cpp         # Multi-object tracking tests
├── filter/
│   ├── test_measurement_update.cpp           # Filter-specific update tests
│   ├── test_sequential_update.cpp            # Sequential update tests
│   ├── test_block_update.cpp                 # Block update tests
│   └── test_update_mode_selection.cpp        # Auto mode selection tests
└── data/
    ├── ground_truth.csv                      # Generated test data
    └── measurements.csv                      # Generated test data
```

### Update single_object_tracking.cpp

- Add measurement update demonstration
- Show both Kalman and Information filter usage
- Include observation model examples
- Demonstrate the complete tracking cycle (predict + update)

## Mathematical Formulation

### Kalman Filter Measurement Update

**Standard Form:**
```
K = P * H' * inv(H * P * H' + R)
x = x + K * (z - H * x)
P = (I - K * H) * P
```

**UDU Factored Form:**
```
// Sequential rank-1 updates using Agee-Turner algorithm
for each measurement component:
    P.rank1Update(...)
```

### Information Filter Measurement Update

**Standard Form:**
```
i = H' * inv(R) * z
I = H' * inv(R) * H
y = y + i
Y = Y + I
```

**UDU Factored Form:**
```
// Based on D'Souza and Zanetti (2019)
// Sequential rank-1 updates for information matrix
```

## Literature References

1. **D'Souza and Zanetti (2019)**: Information Formulation of the UDU Kalman Filter
2. **Bierman (1977)**: Factorization Methods for Discrete Sequential Estimation
3. **Thornton (1976)**: Triangular Covariance Factorizations for Kalman Filtering
4. **Agee and Turner (1972)**: Optimal recursive algorithms for covariance matrices

## Success Criteria

1. **Architectural Consistency**: Prediction and update follow the same pattern
2. **Correct Representation**: Information filter properly works in information space
3. **Unified Interface**: Measurement update works consistently across all filter/covariance combinations
4. **Numerical Stability**: UDU implementations maintain factorization and stability
5. **Comprehensive Testing**: All combinations tested with proper coverage
6. **Documentation**: Complete Doxygen documentation for all new components
7. **RangeBearingDoppler Support**: Observation model with Doppler (radial velocity) measurement and velocity derivatives in Jacobian
8. **Modular Jacobian Construction**: Row-based Jacobian computation that adapts to available state variables
9. **Sequential and Block Updates**: Both sequential (scalar) and block (vector) update workflows available
10. **State Variable Compatibility**: Observation models work with motion models having varying state dimensions (position only, position+velocity, position+velocity+acceleration)

## Timeline Estimation

1. **Architectural Refactoring**: 2-3 days
2. **Information Filter Fix**: 2-3 days
3. **State Trait System**: 1-2 days
4. **Observation Model Interface**: 2-3 days
5. **Concrete Observation Models**:
   - Position: 0.5 day
   - Velocity: 0.5 day
   - RangeBearing: 1 day
   - RangeBearingDoppler: 2 days
6. **GenericUpdate Implementation**: 3-4 days
7. **Filter Measurement Updates**: 2-3 days
8. **Motion Model Integration**: 1-2 days
9. **Testing Infrastructure**: 4-5 days
10. **Example Updates**: 1 day
11. **Documentation**: 2-3 days

**Total**: 23-32 days

## Risk Mitigation

1. **Numerical Stability**: Extensive testing with edge cases
2. **Performance**: Profile UDU implementations for efficiency
3. **Integration**: Incremental testing at each step
4. **Literature Compliance**: Cross-check with academic references
5. **Pattern Consistency**: Ensure all components follow the same architectural pattern

## Architecture Benefits

### Consistency
- Prediction and update follow identical patterns
- All motion models benefit from the same capabilities automatically
- Reduced code duplication and maintenance burden

### Extensibility
- Easy to add new motion models
- Simple to add new observation types
- Clear separation between filter logic and motion model logic

### Maintainability
- Single point of implementation for prediction and update logic
- Clear architectural boundaries
- Consistent API across all components