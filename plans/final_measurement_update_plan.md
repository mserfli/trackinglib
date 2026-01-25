# Measurement Update Implementation Plan - Final

## Overview

This plan provides a comprehensive approach to implementing measurement updates for the trackinglib library, including an architectural refactoring step to improve consistency between prediction and update patterns.

## Step 0: Architectural Refactoring for Consistency

### Prediction Architecture Refactoring

**Objective**: Refactor the prediction architecture to match the proposed update pattern for better consistency and maintainability.

#### Current Issues:
1. **Code Duplication**: Each motion model (CV, CA) implements its own predict methods
2. **Inconsistent Pattern**: Prediction uses GenericPredict but motion models still have individual predict implementations
3. **Maintenance Burden**: Adding new motion models requires duplicating prediction logic

#### Proposed Solution:
Move prediction logic to ExtendedMotionModel following the same pattern as proposed for updates.

#### Required Changes:

**Files**: 
- `include/trackingLib/motion/imotion_model.h`
- `include/trackingLib/motion/extended_motion_model.h`
- `include/trackingLib/motion/motion_model_cv.h`
- `include/trackingLib/motion/motion_model_ca.h`
- `include/trackingLib/motion/generic_predict.h`
- `include/trackingLib/motion/generic_predict.hpp`

**Implementation**:

```cpp
// In ExtendedMotionModel
template <typename MotionModel_, typename MotionModelTrait_>
class ExtendedMotionModel : ... 
{
    // Add predict method to ExtendedMotionModel
    void predict(const value_type dt, const KalmanFilterType& filter, const EgoMotionType& egoMotion)
    {
        generic::Predict<MotionModel_, CovarianceMatrixPolicy_>::run(
            dt, filter, egoMotion, static_cast<MotionModel_&>(*this));
    }
    
    void predict(const value_type dt, const InformationFilterType& filter, const EgoMotionType& egoMotion)
    {
        generic::Predict<MotionModel_, CovarianceMatrixPolicy_>::run(
            dt, filter, egoMotion, static_cast<MotionModel_&>(*this));
    }
};

// Remove individual predict methods from MotionModelCV and MotionModelCA
// They will inherit from ExtendedMotionModel
```

**Benefits**:
- **Consistency**: Prediction follows same pattern as update
- **Reduced Duplication**: No need for individual predict methods in each motion model
- **Easier Maintenance**: New motion models automatically get prediction capability
- **Better Separation**: GenericPredict handles all prediction logic

**Files to Update**:
- Remove predict methods from MotionModelCV and MotionModelCA
- Update GenericPredict to work with ExtendedMotionModel pattern
- Modify tests to use the new pattern
- Update example code to demonstrate the consistent pattern

## Step 1: Information Filter Representation Fix

### Problem Analysis

**Current Issue**: The information filter currently estimates the state x directly, but according to literature, in information space we should estimate y = Y * x (information vector) where Y is the information matrix (inverse of covariance P).

**Current Implementation**: In `generic_predict_common.hpp`, the state prediction is handled identically for both Kalman and Information filters:

```cpp
// transform posteriori state into current frame
underlying.compensateEgoMotion(data.Ge, data.Go, egoMotion);

// apply state transition in current frame
underlying.computeA(data.A, dt);
underlying.applyProcessModel(dt);
```

This needs to be specialized for the information filter with proper ego motion compensation in information space.

### Solution Design

#### 1. Filter-Specific State Prediction

**Files**: `include/trackingLib/motion/generic_predict_common.h`, `include/trackingLib/motion/generic_predict_common.hpp`

**Changes**:

```cpp
// Modify PredictCommon::run to be filter-specific
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class PredictCommon
{
public:
    // ... existing code ...
    
    // Add filter-specific state prediction methods
    static void predictStateForKalmanFilter(
        MotionModel_& underlying,
        const value_type dt,
        const EgoMotionType& egoMotion)
    {
        // Current implementation for Kalman filter
        underlying.compensateEgoMotion(data.Ge, data.Go, egoMotion);
        underlying.computeA(data.A, dt);
        underlying.applyProcessModel(dt);
    }
    
    static void predictStateForInformationFilter(
        MotionModel_& underlying,
        Storage& data,
        const value_type dt,
        const EgoMotionType& egoMotion)
    {
        // Information filter state prediction with ego motion compensation
        
        // Step 1: Compute ego motion compensation matrices
        underlying.compensateEgoMotion(data.Ge, data.Go, egoMotion);
        
        // Step 2: Compute state transition matrix
        underlying.computeA(data.A, dt);
        
        // Step 3: Apply state transition in information space
        // For information filter: y_k+1 = inv(Go)' * (inv(A)' * y_k - inv(A)' * Y_k * Ge * ego_motion)
        
        auto& y = underlying.getVecForInternalUse();
        auto& Y = underlying.getCovForInternalUse();
        
        // Compute the combined transformation: inv(Go)' * inv(A)'
        auto invAGoT = (data.A * data.Go).transpose().inverse();
        
        // Apply ego motion compensation in information space
        auto ego_motion_compensation = invAGoT * Y * data.Ge * egoMotion.getDisplacementCog().vec;
        
        // Apply state transition: y_k+1 = inv(Go)' * inv(A)' * y_k - ego_motion_compensation
        y = invAGoT * y - ego_motion_compensation;
        
        // Step 4: Apply process model (state transition)
        // Note: applyProcessModel needs to be modified for information space
        // or replaced with filter-specific state prediction
    }
};
```

#### 2. Filter-Specific Prediction Logic

**Files**: `include/trackingLib/motion/generic_predict.h`, `include/trackingLib/motion/generic_predict.hpp`

**Changes**:

```cpp
// Modify GenericPredict to use filter-specific state prediction
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class Predict
{
public:
    void run(const value_type dt, const KalmanFilterType& filter, const EgoMotionType& egoMotion)
    {
        static typename BasePredictCommon::Storage data{};
        
        auto& underlying = static_cast<MotionModel_&>(*this);
        
        // Use Kalman filter specific state prediction
        BasePredictCommon::predictStateForKalmanFilter(underlying, dt, egoMotion);
        
        // Compute process noise matrices
        underlying.computeQ(data.Q, dt);
        underlying.computeG(data.G, dt);
        
        // Apply covariance prediction
        auto& P = underlying.getCovForInternalUse();
        filter.predictCovariance(P, data.A, data.G, data.Q);
    }
    
    void run(const value_type dt, const InformationFilterType& filter, const EgoMotionType& egoMotion)
    {
        static typename BasePredictCommon::Storage data{};
        
        auto& underlying = static_cast<MotionModel_&>(*this);
        
        // Use Information filter specific state prediction
        BasePredictCommon::predictStateForInformationFilter(underlying, data, dt, egoMotion);
        
        // Compute process noise matrices
        underlying.computeQ(data.Q, dt);
        underlying.computeG(data.G, dt);
        
        // Apply information matrix prediction
        auto& Y = underlying.getCovForInternalUse();
        filter.predictCovariance(Y, data.A, data.G, data.Q);
    }
};
```

#### 3. Motion Model Modifications

**Files**: 
- `include/trackingLib/motion/motion_model_cv.h`
- `include/trackingLib/motion/motion_model_ca.h`

**Changes**:

```cpp
// Modify applyProcessModel to be filter-aware
template <typename CovarianceMatrixPolicy_>
class MotionModelCV : ... 
{
public:
    // ... existing methods ...
    
    // Modify applyProcessModel to handle both state representations
    void applyProcessModel(const value_type dt, FilterType filterType)
    {
        if (filterType == FilterType::KalmanFilter)
        {
            // Current implementation for state space
            const StateVec& stateVec = this->getVec();
            this->operator[](StateDefCV::X) += dt * stateVec.at_unsafe(StateDefCV::VX);
            this->operator[](StateDefCV::Y) += dt * stateVec.at_unsafe(StateDefCV::VY);
        }
        else // FilterType::InformationFilter
        {
            // Information space: state transition is handled in predictStateForInformationFilter
            // No additional state transition needed here
        }
    }
    
    // Overload for backward compatibility
    void applyProcessModel(const value_type dt)
    {
        applyProcessModel(dt, FilterType::KalmanFilter);
    }
};
```

#### 4. Filter Type Tracking

**Files**: `include/trackingLib/motion/extended_motion_model.h`

**Changes**:

```cpp
// Add filter type tracking to ExtendedMotionModel
template <typename MotionModel_, typename MotionModelTrait_>
class ExtendedMotionModel : ... 
{
    // Track which filter is being used
    enum class ActiveFilter { Kalman, Information };
    ActiveFilter active_filter_ = ActiveFilter::Kalman;
    
    // Update predict methods to set filter type
    void predict(const value_type dt, const KalmanFilterType& filter, const EgoMotionType& egoMotion)
    {
        active_filter_ = ActiveFilter::Kalman;
        generic::Predict<MotionModel_, CovarianceMatrixPolicy_>::run(
            dt, filter, egoMotion, static_cast<MotionModel_&>(*this));
    }
    
    void predict(const value_type dt, const InformationFilterType& filter, const EgoMotionType& egoMotion)
    {
        active_filter_ = ActiveFilter::Information;
        generic::Predict<MotionModel_, CovarianceMatrixPolicy_>::run(
            dt, filter, egoMotion, static_cast<MotionModel_&>(*this));
    }
    
    // Add method to get current filter type
    ActiveFilter getActiveFilter() const { return active_filter_; }
};
```

### Implementation Strategy

#### Phase 1: Core Infrastructure
1. **Modify PredictCommon** to add filter-specific state prediction methods
2. **Update GenericPredict** to use filter-specific prediction logic
3. **Add filter type tracking** to ExtendedMotionModel
4. **Modify motion models** to handle both state representations

#### Phase 2: Integration
1. **Update motion models** to use the new prediction logic
2. **Modify tests** to verify information space state prediction with ego motion
3. **Update example code** to demonstrate the information filter usage

#### Phase 3: Validation
1. **Numerical verification** of state transitions with ego motion
2. **Consistency checks** between state space and information space representations
3. **Performance testing** of the new implementation

### Mathematical Formulation

**State Space Prediction (Kalman Filter) with Ego Motion:**
```
// Step 1: Ego motion compensation
x_compensated = Go * x_k + Ge * ego_motion

// Step 2: State transition
x_k+1 = A * x_compensated

// Step 3: Covariance prediction
P_k+1 = A * Go * P_k * Go' * A' + A * Ge * Pe * Ge' * A' + A * P_k * A' + Q_k
```

**Information Space Prediction (Information Filter) with Ego Motion:**
```
// Step 1: Compute transformation matrices
invAGoT = inv((A * Go)')

// Step 2: Ego motion compensation in information space
ego_motion_compensation = invAGoT * Y_k * Ge * ego_motion

// Step 3: State transition in information space
y_k+1 = invAGoT * y_k - ego_motion_compensation

// Step 4: Information matrix prediction
Y_k+1 = inv(Go)' * inv(A)' * Y_k * inv(A) * inv(Go) + ...
```

### Success Criteria

1. **Correct State Transition**: Information filter state prediction with ego motion follows proper information space mathematics
2. **Numerical Stability**: State transitions maintain numerical stability in information space
3. **Consistency**: State space and information space representations are numerically consistent
4. **Performance**: No significant performance degradation compared to current implementation
5. **Test Coverage**: Comprehensive tests verify all state transition scenarios with ego motion

### Timeline Estimation

1. **Core Infrastructure**: 4-5 days
2. **Integration**: 3-4 days  
3. **Validation**: 3-4 days

**Total**: 10-13 days

This refined plan specifically addresses the ego motion compensation in information space and provides a clear path to implement the proper information space representation while maintaining compatibility with the existing architecture. The solution replaces the generic state prediction with filter-specific implementations that properly handle the different mathematical requirements of Kalman and Information filters.

## Step 2: Observation Model Framework

### Observation Model Interface

```cpp
template <typename MeasurementType, typename CovarianceMatrixPolicy_>
class IObservationModel
{
public:
    virtual ~IObservationModel() = default;
    
    // Get measurement vector
    virtual const MeasurementType& getMeasurement() const = 0;
    
    // Get measurement covariance
    virtual const typename CovarianceMatrixPolicy_::template Instantiate<MeasurementType::Size>& 
        getCovariance() const = 0;
    
    // Compute Jacobian for this observation type
    template <typename StateVector>
    void computeJacobian(
        math::Matrix<typename CovarianceMatrixPolicy_::value_type, 
                     MeasurementType::Size, 
                     StateVector::Size>& H,
        const StateVector& x) const;
};
```

### Concrete Observation Models

```cpp
// Position Observation Model
template <typename CovarianceMatrixPolicy_>
class PositionObservationModel 
    : public IObservationModel<
          math::Vector<typename CovarianceMatrixPolicy_::value_type, 2>, 
          CovarianceMatrixPolicy_>
{
public:
    using value_type = typename CovarianceMatrixPolicy_::value_type;
    
    explicit PositionObservationModel(
        const math::Vector<value_type, 2>& measurement,
        const CovarianceMatrixType<2>& R)
        : measurement_(measurement), R_(R) {}
    
    const math::Vector<value_type, 2>& getMeasurement() const override { return measurement_; }
    const CovarianceMatrixType<2>& getCovariance() const override { return R_; }
    
    template <typename StateVector>
    void computeJacobian(
        math::Matrix<value_type, 2, StateVector::Size>& H,
        const StateVector& x) const override
    {
        // For position observation, Jacobian is identity for position components
        H.setZero();
        H(0, StateDef::X) = 1.0;
        H(1, StateDef::Y) = 1.0;
    }
    
private:
    math::Vector<value_type, 2> measurement_;
    CovarianceMatrixType<2> R_;
};

// Range-Bearing Observation Model
template <typename CovarianceMatrixPolicy_>
class RangeBearingObservationModel 
    : public IObservationModel<
          math::Vector<typename CovarianceMatrixPolicy_::value_type, 2>, 
          CovarianceMatrixPolicy_>
{
public:
    using value_type = typename CovarianceMatrixPolicy_::value_type;
    
    explicit RangeBearingObservationModel(
        value_type range, value_type bearing,
        const CovarianceMatrixType<2>& R)
        : measurement_(range, bearing), R_(R) {}
    
    const math::Vector<value_type, 2>& getMeasurement() const override { return measurement_; }
    const CovarianceMatrixType<2>& getCovariance() const override { return R_; }
    
    value_type getRange() const { return measurement_(0); }
    value_type getBearing() const { return measurement_(1); }
    
    template <typename StateVector>
    void computeJacobian(
        math::Matrix<value_type, 2, StateVector::Size>& H,
        const StateVector& x) const override
    {
        // Nonlinear Jacobian for range-bearing observation
        const value_type dx = x(StateDef::X) - sensor_x_;
        const value_type dy = x(StateDef::Y) - sensor_y_;
        const value_type r_squared = dx * dx + dy * dy;
        const value_type r = std::sqrt(r_squared);
        
        // Range Jacobian
        H(0, StateDef::X) = dx / r;
        H(0, StateDef::Y) = dy / r;
        H(0, StateDef::VX) = 0.0;
        H(0, StateDef::VY) = 0.0;
        
        // Bearing Jacobian
        H(1, StateDef::X) = -dy / r_squared;
        H(1, StateDef::Y) = dx / r_squared;
        H(1, StateDef::VX) = 0.0;
        H(1, StateDef::VY) = 0.0;
    }
    
private:
    math::Vector<value_type, 2> measurement_;
    CovarianceMatrixType<2> R_;
    value_type sensor_x_ = 0.0;
    value_type sensor_y_ = 0.0;
};
```

## Step 3: GenericUpdate Implementation

```cpp
template <typename MotionModel_, typename CovarianceMatrixPolicy_>
class GenericUpdate
{
public:
    using value_type = typename CovarianceMatrixPolicy_::value_type;
    using KalmanFilterType = filter::KalmanFilter<CovarianceMatrixPolicy_>;
    using InformationFilterType = filter::InformationFilter<CovarianceMatrixPolicy_>;
    
    // Measurement update method
    template <typename ObservationModel>
    static void run(const ObservationModel& observationModel, const KalmanFilterType& filter, MotionModel_& motionModel);
    
    template <typename ObservationModel>
    static void run(const ObservationModel& observationModel, const InformationFilterType& filter, MotionModel_& motionModel);
};
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

### ExtendedMotionModel Update Method

```cpp
template <typename MotionModel_, typename MotionModelTrait_>
class ExtendedMotionModel : ... 
{
    // Add update method to ExtendedMotionModel
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, const KalmanFilterType& filter)
    {
        generic::Update<MotionModel_, CovarianceMatrixPolicy_>::run(
            observationModel, filter, static_cast<MotionModel_&>(*this));
    }
    
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, const InformationFilterType& filter)
    {
        generic::Update<MotionModel_, CovarianceMatrixPolicy_>::run(
            observationModel, filter, static_cast<MotionModel_&>(*this));
    }
};
```

### IMotionModel Interface Extension

```cpp
template <typename CovarianceMatrixPolicy_>
class IMotionModel : public ... 
{
    // ... existing methods ...
    
    // Add update methods to interface
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, const KalmanFilterType& filter) = 0;
    
    template <typename ObservationModel>
    void update(const ObservationModel& observationModel, const InformationFilterType& filter) = 0;
};
```

## Step 6: Testing Infrastructure

### Test Files to Create/Update

- `tests/motion/test_measurement_update.cpp`
- `tests/filter/test_measurement_update.cpp`
- Updates to `tests/motion/test_motion_model_cv.cpp`
- Updates to `tests/motion/test_motion_model_ca.cpp`

### Test Coverage

- Kalman filter measurement update (full and factored)
- Information filter measurement update (full and factored)
- Observation model Jacobians
- Integration with motion models
- Numerical stability and edge cases

## Step 7: Example Updates

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

## Timeline Estimation

1. **Architectural Refactoring**: 2-3 days
2. **Information Filter Fix**: 2-3 days
3. **Observation Models**: 2-3 days
4. **GenericUpdate Implementation**: 3-4 days
5. **Filter Measurement Updates**: 2-3 days
6. **Motion Model Integration**: 1-2 days
7. **Testing**: 3-4 days
8. **Example Updates**: 1 day
9. **Documentation**: 1-2 days

**Total**: 18-25 days

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