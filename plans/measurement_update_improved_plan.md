# Measurement Update Implementation Plan - Improved

## Overview
This plan addresses the issues with the previous measurement update approach and provides a more robust design for observation models.

## Key Design Decisions

### 1. Observation Model Interface (IObservationModel)

**Problem**: The previous approach tried to define getters for accessors in the base interface, but these are only known by concrete implementations.

**Solution**: Make IObservationModel a pure abstract interface with only the essential methods that all observation models must implement:

```cpp
class IObservationModel {
public:
    virtual ~IObservationModel() = default;
    
    // Core update methods that all observation models must implement
    virtual void update(FilterType& filter, const MeasurementType& measurement) = 0;
    virtual void updateCovariance(CovarianceMatrixType& P, const MatrixType& H, const MatrixType& R) = 0;
    
    // Factory method for creating observation models with specific measurements
    template <typename... Args>
    static std::unique_ptr<IObservationModel> create(Args&&... args);
};
```

### 2. Extended Observation Model (ExtendedObservationModel)

**Problem**: The previous approach tried to inherit from IObservationModel and add concrete accessors, but observation models have completely different accessor patterns.

**Solution**: Use CRTP pattern similar to motion models but with a different approach:

```cpp
template <typename ConcreteObservationModel, typename MeasurementType, typename CovarianceMatrixType, typename FloatType, sint32 DimZ>
class ExtendedObservationModel : public IObservationModel {
public:
    // Measurement access through concrete model
    const MeasurementType& getMeasurement() const {
        return static_cast<const ConcreteObservationModel*>(this)->getMeasurementImpl();
    }
    
    // Covariance access through concrete model  
    const CovarianceMatrixType& getCovariance() const {
        return static_cast<const ConcreteObservationModel*>(this)->getCovarianceImpl();
    }
    
    // Update methods delegate to concrete implementation
    void update(FilterType& filter, const MeasurementType& measurement) override {
        static_cast<ConcreteObservationModel*>(this)->updateImpl(filter, measurement);
    }
    
    void updateCovariance(CovarianceMatrixType& P, const MatrixType& H, const MatrixType& R) override {
        static_cast<ConcreteObservationModel*>(this)->updateCovarianceImpl(P, H, R);
    }
};
```

### 3. Concrete Observation Models

**Problem**: The previous approach didn't properly handle the different accessor patterns for different observation types.

**Solution**: Create specific observation models with their own accessors and initialization:

#### Position Observation Model
```cpp
template <typename CovarianceMatrixType, typename FloatType>
class PositionObservationModel : 
    public ExtendedObservationModel<PositionObservationModel<CovarianceMatrixType, FloatType>, 
                                   Vector<FloatType, 2>, 
                                   CovarianceMatrixType, 
                                   FloatType, 
                                   2> {
public:
    // Constructor takes measurement directly
    explicit PositionObservationModel(const Vector<FloatType, 2>& measurement, 
                                     const CovarianceMatrixType& R)
        : measurement_(measurement), R_(R) {}
    
    // Implementation-specific accessors
    const Vector<FloatType, 2>& getMeasurementImpl() const { return measurement_; }
    const CovarianceMatrixType& getCovarianceImpl() const { return R_; }
    
    // Implementation-specific update methods
    void updateImpl(FilterType& filter, const Vector<FloatType, 2>& measurement) {
        // Update logic specific to position observations
    }
    
    void updateCovarianceImpl(CovarianceMatrixType& P, const MatrixType& H, const MatrixType& R) {
        // Covariance update logic for position observations
    }
    
private:
    Vector<FloatType, 2> measurement_;
    CovarianceMatrixType R_;
};
```

#### Range-Bearing Observation Model
```cpp
template <typename CovarianceMatrixType, typename FloatType>
class RangeBearingObservationModel : 
    public ExtendedObservationModel<RangeBearingObservationModel<CovarianceMatrixType, FloatType>, 
                                   Vector<FloatType, 2>, 
                                   CovarianceMatrixType, 
                                   FloatType, 
                                   2> {
public:
    // Constructor takes measurement directly
    explicit RangeBearingObservationModel(FloatType range, FloatType bearing, 
                                         const CovarianceMatrixType& R)
        : measurement_(range, bearing), R_(R) {}
    
    // Implementation-specific accessors
    const Vector<FloatType, 2>& getMeasurementImpl() const { return measurement_; }
    const CovarianceMatrixType& getCovarianceImpl() const { return R_; }
    
    // Convenience accessors for range-bearing specific values
    FloatType getRange() const { return measurement_(0); }
    FloatType getBearing() const { return measurement_(1); }
    
    // Implementation-specific update methods
    void updateImpl(FilterType& filter, const Vector<FloatType, 2>& measurement) {
        // Update logic specific to range-bearing observations
    }
    
    void updateCovarianceImpl(CovarianceMatrixType& P, const MatrixType& H, const MatrixType& R) {
        // Covariance update logic for range-bearing observations
    }
    
private:
    Vector<FloatType, 2> measurement_;
    CovarianceMatrixType R_;
};
```

### 4. Filter Integration

The filter classes will work with the observation models through the IObservationModel interface:

```cpp
// In KalmanFilter class
void update(StateVector& x, CovarianceMatrixType& P, IObservationModel& observationModel) {
    // Get measurement and covariance from observation model
    auto measurement = observationModel.getMeasurement();
    auto R = observationModel.getCovariance();
    
    // Compute Jacobian H based on observation type
    auto H = computeJacobian(x, observationModel);
    
    // Delegate covariance update to observation model
    observationModel.updateCovariance(P, H, R);
    
    // Update state using observation model
    observationModel.update(*this, measurement);
}
```

## Implementation Steps

### Step 1: Create IObservationModel Interface
- Define pure abstract interface in `include/trackingLib/motion/iobservation_model.h`
- Include only essential virtual methods
- Add factory method pattern

### Step 2: Create ExtendedObservationModel Base Class
- Implement CRTP base class in `include/trackingLib/motion/extended_observation_model.h`
- Provide generic access to measurement and covariance
- Delegate all operations to concrete implementations

### Step 3: Implement Concrete Observation Models
- Create `PositionObservationModel` for direct position measurements
- Create `RangeBearingObservationModel` for range/bearing measurements
- Add any other specific observation types needed

### Step 4: Update Filter Classes
- Modify KalmanFilter to work with IObservationModel interface
- Add support for different observation types
- Ensure covariance updates are handled correctly

### Step 5: Add Test Coverage
- Create comprehensive tests for each observation model
- Test integration with different filter types
- Verify numerical stability and correctness

## Benefits of This Approach

1. **Proper Separation of Concerns**: Each observation model handles its own specific logic
2. **Type Safety**: Compile-time checking of observation dimensions and types
3. **Flexibility**: Easy to add new observation types without modifying existing code
4. **Consistency**: Follows the same CRTP pattern used successfully in motion models
5. **Better Initialization**: Measurements are set at construction time, not through update methods

## Comparison to Motion Models

| Aspect | Motion Models | Observation Models |
|--------|--------------|-------------------|
| **Base Interface** | IMotionModel | IObservationModel |
| **CRTP Base** | ExtendedMotionModel | ExtendedObservationModel |
| **Accessor Pattern** | Standardized (X, VX, AX, etc.) | Model-specific (varies by observation type) |
| **Measurement Handling** | Built into prediction | Handled through constructor/interface |
| **Update Focus** | State prediction | Measurement incorporation |

This design properly handles the fundamental difference that observation models have completely different accessor patterns while maintaining the benefits of the CRTP approach.