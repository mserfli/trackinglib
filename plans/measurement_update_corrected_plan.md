# Measurement Update Implementation Plan - Corrected Design

## Overview

This plan outlines the implementation of measurement update functionality for the trackinglib library, following the exact CRTP pattern used in the prediction system. The design creates an `Observer` class hierarchy that mirrors the `Predict` pattern and integrates with motion models through an `ExtendedObservationModel` layer.

## Current Architecture Analysis

### Prediction Pattern (Reference)
```cpp
// Current prediction architecture:
// 1. IMotionModel - Base interface
// 2. ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size> - CRTP base
// 3. MotionModelCV/MotionModelCA - Concrete motion models
// 4. generic::Predict<MotionModel, FloatType, CovarianceMatrixType> - CRTP mixin
```

### StateMem Pattern (Reference)
```cpp
// StateMem stores state vector and covariance:
template <typename CovarianceMatrixType_, typename FloatType_, sint32 Size_>
class StateMem {
    Vector<FloatType_, Size_> x_;
    CovarianceMatrixType_ P_;
};

// ExtendedMotionModel inherits from both IMotionModel and StateMem:
template <typename MotionModel_, typename CovarianceMatrixType_, typename FloatType_, sint32 Size_>
class ExtendedMotionModel : public IMotionModel<FloatType_>, public StateMem<CovarianceMatrixType_, FloatType_, Size_> {
    // CRTP pattern
};
```

## Corrected Design

### 1. Observation Model Interface

```cpp
// include/trackingLib/observation/iobservation_model.h
template <typename FloatType_>
class IObservationModel {
public:
    virtual ~IObservationModel() = default;
    
    // Observation accessors
    virtual FloatType_ getRange() const = 0;
    virtual FloatType_ getAzimuth() const = 0;
    virtual FloatType_ getElevation() const = 0;
    virtual FloatType_ getX() const = 0;
    virtual FloatType_ getY() const = 0;
    virtual FloatType_ getZ() const = 0;
    
    // Observation model methods
    virtual void update(const Vector<FloatType_, DimZ>& z) = 0;
    virtual void updateCovariance(const DiagonalMatrix<FloatType_, DimR>& R) = 0;
    
    // Filter-specific update methods
    template <typename MotionModel_, typename CovarianceMatrixType_>
    virtual void updateKalmanFilter(MotionModel_& motionModel, const Vector<FloatType_, DimZ>& z, const DiagonalMatrix<FloatType_, DimR>& R) = 0;
    
    template <typename MotionModel_, typename CovarianceMatrixType_>
    virtual void updateInformationFilter(MotionModel_& motionModel, const Vector<FloatType_, DimZ>& z, const DiagonalMatrix<FloatType_, DimR>& R) = 0;
};
```

### 2. Extended Observation Model (Following StateMem Pattern)

```cpp
// include/trackingLib/observation/extended_observation_model.h
template <typename ObservationModel_, typename CovarianceMatrixType_, typename FloatType_, sint32 Size_>
class ExtendedObservationModel : public IObservationModel<FloatType_>, public ObservationMem<CovarianceMatrixType_, FloatType_, Size_> {
public:
    // CRTP pattern - cast to derived class
    ObservationModel_& derived() { return static_cast<ObservationModel_&>(*this); }
    const ObservationModel_& derived() const { return static_cast<const ObservationModel_&>(*this); }
    
    // Factory methods for convenient initialization
    static ObservationModel_ FromLists(
        const std::initializer_list<FloatType_>& z_list,
        const std::initializer_list<FloatType_>& R_list
    );
    
    // Common observation accessors
    FloatType_ getRange() const override { return derived().getRange(); }
    FloatType_ getAzimuth() const override { return derived().getAzimuth(); }
    FloatType_ getElevation() const override { return derived().getElevation(); }
    FloatType_ getX() const override { return derived().getX(); }
    FloatType_ getY() const override { return derived().getY(); }
    FloatType_ getZ() const override { return derived().getZ(); }
};
```

### 3. Observation Memory (Following StateMem Pattern)

```cpp
// include/trackingLib/observation/observation_mem.h
template <typename CovarianceMatrixType_, typename FloatType_, sint32 Size_>
class ObservationMem {
protected:
    Vector<FloatType_, Size_> z_;
    CovarianceMatrixType_ R_;
    
public:
    // Accessors
    const Vector<FloatType_, Size_>& getObservation() const { return z_; }
    const CovarianceMatrixType_& getObservationCovariance() const { return R_; }
    
    // Mutators
    void setObservation(const Vector<FloatType_, Size_>& z) { z_ = z; }
    void setObservationCovariance(const CovarianceMatrixType_& R) { R_ = R; }
};
```

### 4. Observer Class Hierarchy

```cpp
// include/trackingLib/observation/observer.h
namespace generic {

template <typename ObservationModel_, typename FloatType_, typename CovarianceMatrixType_>
class Observer {
public:
    // Main update method
    void update(
        ObservationModel_& observationModel,
        typename ObservationModel_::MotionModelType& motionModel,
        const Vector<FloatType_, ObservationModel_::DimZ>& z,
        const DiagonalMatrix<FloatType_, ObservationModel_::DimR>& R
    ) {
        // Update observation
        observationModel.update(z);
        observationModel.updateCovariance(R);
        
        // Dispatch to filter-specific update
        updateFilterSpecific(observationModel, motionModel);
    }
    
    // Common calculations for all filters
    void computeCommonMatrices(
        const ObservationModel_& observationModel,
        const typename ObservationModel_::MotionModelType& motionModel,
        SquareMatrix<FloatType_, ObservationModel_::DimZ>& H,
        Matrix<FloatType_, ObservationModel_::DimX, ObservationModel_::DimZ>& K
    ) const {
        // Compute observation matrix H
        observationModel.computeH(H);
        
        // Compute Kalman gain K (common to all filters)
        // K = P * H' * (H * P * H' + R)^-1
        // For factored covariance: use Agee-Turner rank-1 update
        
        if constexpr (std::is_same_v<CovarianceMatrixType_, CovarianceMatrixFactored<FloatType_, ObservationModel_::DimX>>) {
            // Use Agee-Turner rank-1 update for factored covariance
            observationModel.computeKalmanGainFactored(K, H, motionModel.getCovariance());
        } else {
            // Standard Kalman gain calculation
            observationModel.computeKalmanGainStandard(K, H, motionModel.getCovariance(), observationModel.getObservationCovariance());
        }
    }
    
private:
    // Filter-specific dispatch
    void updateFilterSpecific(
        ObservationModel_& observationModel,
        typename ObservationModel_::MotionModelType& motionModel
    ) {
        if constexpr (ObservationModel_::FilterType == FilterType::Kalman) {
            updateKalmanFilter(observationModel, motionModel);
        } else if constexpr (ObservationModel_::FilterType == FilterType::Information) {
            updateInformationFilter(observationModel, motionModel);
        }
    }
    
    // Kalman Filter update
    void updateKalmanFilter(
        ObservationModel_& observationModel,
        typename ObservationModel_::MotionModelType& motionModel
    ) {
        // Compute common matrices
        SquareMatrix<FloatType_, ObservationModel_::DimZ> H;
        Matrix<FloatType_, ObservationModel_::DimX, ObservationModel_::DimZ> K;
        computeCommonMatrices(observationModel, motionModel, H, K);
        
        // Update state: x = x + K * (z - H * x)
        Vector<FloatType_, ObservationModel_::DimX> innovation = 
            observationModel.getObservation() - H * motionModel.getState();
        
        motionModel.setState(motionModel.getState() + K * innovation);
        
        // Update covariance: P = (I - K * H) * P
        // For factored covariance: use rank-1 update
        observationModel.updateCovarianceKalman(motionModel, K, H);
    }
    
    // Information Filter update
    void updateInformationFilter(
        ObservationModel_& observationModel,
        typename ObservationModel_::MotionModelType& motionModel
    ) {
        // Compute common matrices
        SquareMatrix<FloatType_, ObservationModel_::DimZ> H;
        Matrix<FloatType_, ObservationModel_::DimX, ObservationModel_::DimZ> K;
        computeCommonMatrices(observationModel, motionModel, H, K);
        
        // Update information matrix and vector
        observationModel.updateInformationFilter(motionModel, H, K);
    }
};

} // namespace generic
```

### 5. Concrete Observation Models

#### Cartesian Observation Model
```cpp
// include/trackingLib/observation/observation_model_cartesian.h
template <typename CovarianceMatrixType_, typename FloatType_>
class ObservationModelCartesian : 
    public ExtendedObservationModel<ObservationModelCartesian<CovarianceMatrixType_, FloatType_>, CovarianceMatrixType_, FloatType_, 2> {
public:
    static constexpr sint32 DimX = 4;  // State dimension (CV model)
    static constexpr sint32 DimZ = 2;  // Observation dimension (x, y)
    static constexpr sint32 DimR = 2;  // Observation noise dimension
    
    using MotionModelType = MotionModelCV<CovarianceMatrixType_, FloatType_>;
    using FilterType = FilterType::Kalman;  // Default, can be overridden
    
    // Observation accessors
    FloatType_ getX() const override { return z_.at(0); }
    FloatType_ getY() const override { return z_.at(1); }
    FloatType_ getRange() const override { return std::sqrt(z_.at(0)*z_.at(0) + z_.at(1)*z_.at(1)); }
    FloatType_ getAzimuth() const override { return std::atan2(z_.at(1), z_.at(0)); }
    FloatType_ getElevation() const override { return 0; } // 2D only
    
    // Observation matrix H for Cartesian observations
    void computeH(SquareMatrix<FloatType_, DimZ>& H) const {
        // H = [I 0] for [x, y] observations from [x, vx, y, vy] state
        H.setIdentity();
    }
    
    // Kalman gain calculation
    void computeKalmanGainStandard(
        Matrix<FloatType_, DimX, DimZ>& K,
        const SquareMatrix<FloatType_, DimZ>& H,
        const CovarianceMatrixType_& P,
        const DiagonalMatrix<FloatType_, DimR>& R
    ) const {
        // Standard Kalman gain: K = P * H' * (H * P * H' + R)^-1
        auto S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();
    }
    
    void computeKalmanGainFactored(
        Matrix<FloatType_, DimX, DimZ>& K,
        const SquareMatrix<FloatType_, DimZ>& H,
        const CovarianceMatrixFactored<FloatType_, DimX>& P
    ) const {
        // Use Agee-Turner rank-1 update for factored covariance
        // Implementation uses the rank1_update utilities
    }
    
    // Covariance update for Kalman filter
    void updateCovarianceKalman(
        MotionModelType& motionModel,
        const Matrix<FloatType_, DimX, DimZ>& K,
        const SquareMatrix<FloatType_, DimZ>& H
    ) const {
        if constexpr (std::is_same_v<CovarianceMatrixType_, CovarianceMatrixFactored<FloatType_, DimX>>) {
            // Factored covariance update using rank-1 update
            auto I_KH = SquareMatrix<FloatType_, DimX>::Identity() - K * H;
            motionModel.setCovariance(I_KH * motionModel.getCovariance() * I_KH.transpose());
        } else {
            // Standard covariance update
            auto I_KH = SquareMatrix<FloatType_, DimX>::Identity() - K * H;
            motionModel.setCovariance(I_KH * motionModel.getCovariance());
        }
    }
    
    // Information filter update
    void updateInformationFilter(
        MotionModelType& motionModel,
        const SquareMatrix<FloatType_, DimZ>& H,
        const Matrix<FloatType_, DimX, DimZ>& K
    ) const {
        // Update information matrix: Y = Y + H' * R^-1 * H
        // Update information vector: y = y + H' * R^-1 * z
        // Implementation depends on whether using full or factored covariance
    }
};
```

#### Polar Observation Model
```cpp
// include/trackingLib/observation/observation_model_polar.h
template <typename CovarianceMatrixType_, typename FloatType_>
class ObservationModelPolar : 
    public ExtendedObservationModel<ObservationModelPolar<CovarianceMatrixType_, FloatType_>, CovarianceMatrixType_, FloatType_, 2> {
public:
    static constexpr sint32 DimX = 4;  // State dimension (CV model)
    static constexpr sint32 DimZ = 2;  // Observation dimension (range, azimuth)
    static constexpr sint32 DimR = 2;  // Observation noise dimension
    
    using MotionModelType = MotionModelCV<CovarianceMatrixType_, FloatType_>;
    using FilterType = FilterType::Kalman;
    
    // Observation accessors
    FloatType_ getRange() const override { return z_.at(0); }
    FloatType_ getAzimuth() const override { return z_.at(1); }
    FloatType_ getElevation() const override { return 0; } // 2D only
    FloatType_ getX() const override { return z_.at(0) * std::cos(z_.at(1)); }
    FloatType_ getY() const override { return z_.at(0) * std::sin(z_.at(1)); }
    FloatType_ getZ() const override { return 0; }
    
    // Observation matrix H for polar observations
    void computeH(SquareMatrix<FloatType_, DimZ>& H) const {
        // H = [dx/dr dx/dθ; dy/dr dy/dθ] evaluated at current state
        FloatType_ r = getRange();
        FloatType_ theta = getAzimuth();
        
        H(0, 0) = std::cos(theta);  // dx/dr
        H(0, 1) = -r * std::sin(theta); // dx/dθ
        H(1, 0) = std::sin(theta);  // dy/dr
        H(1, 1) = r * std::cos(theta);  // dy/dθ
    }
    
    // Rest of implementation similar to Cartesian model
    // but with polar-specific calculations
};
```

## Integration with Motion Models

### Motion Model Interface Extension
```cpp
// Add to IMotionModel interface
template <typename FloatType_>
class IMotionModel {
    // ... existing methods ...
    
    // New methods for measurement update
    template <typename ObservationModel_>
    virtual void update(const ObservationModel_& observationModel) = 0;
    
    template <typename ObservationModel_>
    virtual void updateKalmanFilter(const ObservationModel_& observationModel) = 0;
    
    template <typename ObservationModel_>
    virtual void updateInformationFilter(const ObservationModel_& observationModel) = 0;
};
```

### Motion Model Implementation
```cpp
// Add to MotionModelCV implementation
template <typename CovarianceMatrixType_, typename FloatType_>
class MotionModelCV : public ExtendedMotionModel<MotionModelCV<CovarianceMatrixType_, FloatType_>, CovarianceMatrixType_, FloatType_, 4> {
    // ... existing methods ...
    
    template <typename ObservationModel_>
    void update(const ObservationModel_& observationModel) override {
        generic::Observer<ObservationModel_, FloatType_, CovarianceMatrixType_> observer;
        observer.update(observationModel, *this);
    }
    
    template <typename ObservationModel_>
    void updateKalmanFilter(const ObservationModel_& observationModel) override {
        generic::Observer<ObservationModel_, FloatType_, CovarianceMatrixType_> observer;
        observer.updateKalmanFilter(observationModel, *this);
    }
    
    template <typename ObservationModel_>
    void updateInformationFilter(const ObservationModel_& observationModel) override {
        generic::Observer<ObservationModel_, FloatType_, CovarianceMatrixType_> observer;
        observer.updateInformationFilter(observationModel, *this);
    }
};
```

## Directory Structure

```
include/trackingLib/
└── observation/                    # New observation layer
    ├── iobservation_model.h       # Base interface
    ├── extended_observation_model.h # CRTP base (like ExtendedMotionModel)
    ├── observation_mem.h          # Observation memory (like StateMem)
    ├── observer.h                 # Observer class (like Predict)
    ├── observation_model_cartesian.h # Cartesian observation model
    ├── observation_model_polar.h  # Polar observation model
    ├── observation_model_3d.h     # 3D observation model
    └── contracts/                 # Observation model contracts
```

## Implementation Phases

### Phase 1: Core Infrastructure
1. Create observation memory class (`ObservationMem`)
2. Create extended observation model base (`ExtendedObservationModel`)
3. Create observation model interface (`IObservationModel`)
4. Create observer class hierarchy (`generic::Observer`)
5. Add observation model contracts

### Phase 2: Concrete Observation Models
1. Implement Cartesian observation model
2. Implement Polar observation model  
3. Implement 3D observation model
4. Add factory methods for convenient initialization

### Phase 3: Motion Model Integration
1. Extend `IMotionModel` interface with update methods
2. Implement update methods in `MotionModelCV`
3. Implement update methods in `MotionModelCA`
4. Test integration between motion and observation models

### Phase 4: Filter-Specific Implementations
1. Implement Kalman filter measurement update
2. Implement Information filter measurement update
3. Test both full and factored covariance versions
4. Verify numerical stability

### Phase 5: Testing and Validation
1. Create comprehensive unit tests
2. Test with different observation types
3. Test with different motion models
4. Test with different covariance representations
5. Validate against reference implementations

## Testing Strategy

### Test Coverage Targets
- **Line Coverage**: >90%
- **Branch Coverage**: >85%
- **Test Count**: 50-75 new tests

### Test Categories
1. **Observation Memory Tests**: State storage and access
2. **CRTP Pattern Tests**: Proper inheritance and casting
3. **Cartesian Observation Tests**: H matrix calculation, update logic
4. **Polar Observation Tests**: H matrix calculation, Jacobian accuracy
5. **Integration Tests**: Motion model + observation model interaction
6. **Numerical Stability Tests**: Factored vs full covariance comparison
7. **Edge Case Tests**: Invalid observations, zero covariance, etc.

### Test Files
- `tests/observation/test_observation_mem.cpp`
- `tests/observation/test_observation_model_cartesian.cpp`
- `tests/observation/test_observation_model_polar.cpp`
- `tests/observation/test_observer_integration.cpp`
- `tests/motion/test_motion_observation_integration.cpp`

## Success Criteria

1. **Functional**: All observation models work with all motion models
2. **Numerical**: Results match reference implementations within tolerance
3. **Performance**: No significant performance degradation
4. **Code Quality**: AUTOSAR C++14 compliant, comprehensive documentation
5. **Testing**: >90% line coverage, >85% branch coverage
6. **Integration**: Seamless integration with existing prediction system

## Risk Mitigation

1. **Complexity Risk**: Break implementation into small, testable phases
2. **Numerical Issues**: Extensive testing with edge cases
3. **Integration Problems**: Test each component before full integration
4. **Performance Concerns**: Profile critical sections
5. **Documentation Gaps**: Document each component as implemented

## Timeline Estimation

- **Design & Planning**: 1-2 days (COMPLETED)
- **Core Infrastructure**: 3-5 days
- **Concrete Models**: 2-3 days
- **Integration**: 2-3 days
- **Testing & Validation**: 3-5 days
- **Documentation**: 1-2 days

**Total**: 12-20 days

## Next Steps

1. Create observation memory class
2. Create extended observation model base
3. Create observer class hierarchy
4. Implement Cartesian observation model
5. Integrate with motion models
6. Write comprehensive tests