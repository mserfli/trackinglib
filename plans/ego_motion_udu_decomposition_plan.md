# Ego Motion UDU Decomposition Implementation Plan

## Overview

This plan documents the implementation strategy for calculating ego motion displacement natively as UDU decomposition instead of decomposing full matrices after calculations. The goal is to improve numerical stability and consistency with the library's factored covariance philosophy.

## Current State Analysis

### Current Implementation Issues

1. **Numerical Stability**: Current approach computes full covariance matrices and then decomposes them
2. **Performance**: Extra decomposition step adds computational overhead
3. **Consistency**: Inconsistent with library's focus on factored covariance representations

### Key Requirements

1. **Positive Definiteness**: All covariance matrices must remain positive definite
2. **Numerical Stability**: Avoid operations that can cause numerical instability
3. **Backward Compatibility**: Existing code using full covariance must continue to work
4. **Type Safety**: Compile-time selection of covariance representation

## Proposed Solution

### Templatized EgoMotion Class

```cpp
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType = float32>
class EgoMotion
{
    // Class implementation using CovarianceMatrixType for displacement covariance
};
```

### Mathematical Approach

The displacement covariance calculation follows:
```
P = J * Pin * J^T
```

Where:
- `Pin` = diag([σ_v², σ_a², σ_ω²]) (positive definite diagonal matrix)
- `J` = Jacobian matrix (full rank for valid motion)
- `P` = Resulting covariance matrix (positive definite)

## Implementation Workflow

### Step 1: Baseline Testing

**Objective:** Establish comprehensive baseline tests before any refactoring to ensure numerical equivalence after changes.

**Tasks:**
- [ ] Create unit test file `tests/motion/test_ego_motion.cpp`
- [ ] Implement test cases for current linear motion displacement calculation
- [ ] Implement test cases for current circular motion displacement calculation
- [ ] Add reference test cases with known expected results
- [ ] Test edge cases (small ω, zero acceleration, high velocities)
- [ ] Document expected covariance matrices for regression testing
- [ ] Ensure all baseline tests pass with current implementation

**Test Structure:**

```cpp
// Test fixture for ego motion calculations
class EgoMotionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Common test setup
        motion.v = 10.0f;  // 10 m/s
        motion.a = 2.0f;   // 2 m/s²
        motion.w = 0.1f;   // 0.1 rad/s (small for linear case)
        motion.sv = 0.5f;  // 0.5 m/s std dev
        motion.sa = 0.2f;  // 0.2 m/s² std dev
        motion.sw = 0.05f; // 0.05 rad/s std dev
        
        geometry.distCog2Ego = 1.5f; // 1.5m from COG to ego point
        dt = 0.1f; // 100ms time step
    }
    
    env::EgoMotion<FloatType>::InertialMotion motion{};
    env::EgoMotion<FloatType>::Geometry geometry{};
    FloatType dt = 0.1f;
};

// Test linear motion displacement
TEST_F(EgoMotionTest, LinearMotionDisplacement__Success) {
    auto egoMotion = env::EgoMotion<float32>{motion, geometry, dt};
    
    // Expected displacement values (from derivation)
    const FloatType expected_dx = 1.0005f; // T*v + 0.5*T²*(a - v*w)
    const FloatType expected_dy = 0.0005f; // 0.5*T²*v*w
    const FloatType expected_dpsi = 0.01f; // T*w
    
    // Expected covariance matrix (from J * Pin * J^T)
    const math::SquareMatrix<float32, 3, true> expected_cov{
        {0.250125f, 0.000125f, 0.0f},
        {0.000125f, 0.000125f, 0.0f},
        {0.0f, 0.0f, 0.0001f}
    };
    
    // Verify displacement vector
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(0), expected_dx, 1e-6);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(1), expected_dy, 1e-6);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(2), expected_dpsi, 1e-6);
    
    // Verify covariance matrix
    auto actual_cov = egoMotion.getDisplacementCog().cov();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(actual_cov.at_unsafe(i, j), expected_cov.at_unsafe(i, j), 1e-6);
        }
    }
}

// Test circular motion displacement
TEST_F(EgoMotionTest, CircularMotionDisplacement__Success) {
    // Use larger ω for circular motion
    motion.w = 0.5f; // 0.5 rad/s
    
    auto egoMotion = env::EgoMotion<float32>{motion, geometry, dt};
    
    // Expected displacement values (from circular motion equations)
    const FloatType dphi = motion.w * dt;
    const FloatType sin_dphi_2 = std::sin(dphi/2);
    const FloatType cos_dphi_2 = std::cos(dphi/2);
    const FloatType c = dt * (2*motion.v + motion.a*dt) / dphi * sin_dphi_2;
    const FloatType expected_dx = c * cos_dphi_2;
    const FloatType expected_dy = c * sin_dphi_2;
    const FloatType expected_dpsi = dphi;
    
    // Verify displacement vector
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(0), expected_dx, 1e-6);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(1), expected_dy, 1e-6);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(2), expected_dpsi, 1e-6);
    
    // Verify covariance matrix properties (symmetry, positive definiteness)
    auto actual_cov = egoMotion.getDisplacementCog().cov();
    EXPECT_TRUE(actual_cov.isSymmetric(1e-6));
    EXPECT_TRUE(actual_cov.isPositiveDefinite());
}
```

**Files to Create:**
- `tests/motion/test_ego_motion.cpp` - Main test file
- `tests/motion/reference/ego_motion_reference_values.h` - Reference values header

### Step 2: Refactored Architecture

**Objective:** Create clean separation between Jacobian calculation and covariance computation.

**Tasks:**
- [ ] Implement `calcLinearMotionJacobian()` for linear motion case
- [ ] Implement `calcCircularMotionJacobian()` for circular motion case
- [ ] Implement `calcDisplacementVector()` for common displacement calculation
- [ ] Implement `calcDisplacementCovariance()` with simplified UDU approach
- [ ] Update EgoMotion class to use refactored methods

**Refactored Architecture:**

```cpp
// Refactored EgoMotion class with separated Jacobian calculation
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType = float32>
class EgoMotion
{
public:
    // ... existing members ...
    
    // Common displacement calculation method
    void calcDisplacement()
    {
        // Calculate displacement vector (common for both motion types)
        calcDisplacementVector(_displacementCog, _motion, _dt);
        
        // Calculate covariance using appropriate method based on motion type
        if (isLinearMotion())
        {
            calcDisplacementCovariance(_displacementCog, 
                                     calcLinearMotionJacobian(_motion, _dt), 
                                     _motion);
        }
        else
        {
            calcDisplacementCovariance(_displacementCog, 
                                     calcCircularMotionJacobian(_motion, _dt), 
                                     _motion);
        }
    }
    
    // Static methods for Jacobian calculation (pure functions)
    static auto calcLinearMotionJacobian(const InertialMotion& motion, FloatType dt) 
        -> math::SquareMatrix<FloatType, 3, true>;
    
    static auto calcCircularMotionJacobian(const InertialMotion& motion, FloatType dt)
        -> math::SquareMatrix<FloatType, 3, true>;
    
private:
    // Common displacement vector calculation
    static void calcDisplacementVector(Displacement& displacement,
                                     const InertialMotion& motion,
                                     FloatType dt);
    
    // Common covariance calculation (templatized with simplified UDU)
    void calcDisplacementCovariance(Displacement& displacement,
                                  const math::SquareMatrix<FloatType, 3, true>& J,
                                  const InertialMotion& motion);
};
```

**Simplified UDU Implementation (Integrated from Start):**

```cpp
// Common covariance calculation method with simplified UDU approach
template <template <typename, sint32> class CovarianceMatrixType, typename FloatType>
void EgoMotion<CovarianceMatrixType, FloatType>::calcDisplacementCovariance(
    Displacement& displacement,
    const math::SquareMatrix<FloatType, 3, true>& J,
    const InertialMotion& motion)
{
    if constexpr (std::is_same_v<CovarianceMatrixType<FloatType, 3>, 
                                 CovarianceMatrixFactored<FloatType, 3>>)
    {
        // Simplified UDU implementation using existing interface
        displacement.cov.setIdentity();
        
        // Set diagonal elements to create Pin = diag([σ_v², σ_a², σ_ω²])
        displacement.cov.setDiagonal(0, math::pow<2>(motion.sv));
        displacement.cov.setDiagonal(1, math::pow<2>(motion.sa));
        displacement.cov.setDiagonal(2, math::pow<2>(motion.sw));
        
        // Apply the transformation: P = J * Pin * J^T using apaT
        displacement.cov.apaT(J);
    }
    else
    {
        // Full matrix implementation
        CovarianceMatrixFull<FloatType, 3> fullCov{};
        fullCov.setZeros();
        
        // Set diagonal elements
        fullCov.at_unsafe(0, 0) = math::pow<2>(motion.sv);
        fullCov.at_unsafe(1, 1) = math::pow<2>(motion.sa);
        fullCov.at_unsafe(2, 2) = math::pow<2>(motion.sw);
        
        // Apply transformation
        fullCov.apaT(J);
        
        displacement.cov = std::move(fullCov);
    }
}
```

**Benefits of This Integrated Approach:**

1. **Simplicity**: Uses existing, well-tested interface methods from the beginning
2. **Numerical Stability**: Leverages proven Modified Gram-Schmidt implementation
3. **Consistency**: Follows established patterns in the codebase
4. **Maintainability**: Clean, understandable code structure
5. **Performance**: Efficient computation without manual matrix manipulation

### Step 3: Motion Model Integration

**Objective:** Update motion models to use the refactored, templatized EgoMotion class.

**Tasks:**
- [ ] Update MotionModelCV to use templatized EgoMotion
- [ ] Update MotionModelCA to use templatized EgoMotion
- [ ] Ensure type consistency between motion models and ego motion
- [ ] Update prediction methods to handle both covariance types
- [ ] Add comprehensive integration tests

**Integration Pattern:**

```cpp
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
class MotionModelCV : public ExtendedMotionModel<...>
{
    // Use matching EgoMotion type
    using EgoMotionType = env::EgoMotion<CovarianceMatrixType, FloatType>;
    
    void predict(const FloatType dt,
                 const filter::KalmanFilter<FloatType>& filter,
                 const EgoMotionType& egoMotion) final
    {
        // Type-consistent ego motion compensation
        egoMotion.compensatePosition(/* ... */);
        // ... rest of prediction logic ...
    }
};
```

### Step 4: Comprehensive Testing

**Objective:** Validate the implementation thoroughly and ensure numerical equivalence.

**Tasks:**
- [ ] Add regression tests comparing old vs new implementation
- [ ] Test numerical equivalence between refactored and original code
- [ ] Validate Jacobian calculation methods independently
- [ ] Test displacement consistency between implementations
- [ ] Test covariance equivalence between both approaches
- [ ] Add performance benchmarking tests
- [ ] Test edge cases and numerical stability

**Test Examples:**

```cpp
// Regression test comparing implementations
TEST_F(EgoMotionTest, RefactoredImplementation__NumericalEquivalence) {
    // Test with current implementation
    auto egoMotionOld = env::EgoMotion<float32>{motion, geometry, dt};
    auto oldDisplacement = egoMotionOld.getDisplacementCog();
    
    // Test with refactored implementation
    auto egoMotionNew = env::EgoMotionRefactored<float32>{motion, geometry, dt};
    auto newDisplacement = egoMotionNew.getDisplacementCog();
    
    // Verify displacement vectors match
    EXPECT_NEAR(oldDisplacement.vec.at_unsafe(0), newDisplacement.vec.at_unsafe(0), 1e-6);
    EXPECT_NEAR(oldDisplacement.vec.at_unsafe(1), newDisplacement.vec.at_unsafe(1), 1e-6);
    EXPECT_NEAR(oldDisplacement.vec.at_unsafe(2), newDisplacement.vec.at_unsafe(2), 1e-6);
    
    // Verify covariance matrices match
    auto oldCov = oldDisplacement.cov();
    auto newCov = newDisplacement.cov();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(oldCov.at_unsafe(i, j), newCov.at_unsafe(i, j), 1e-6);
        }
    }
}

// Jacobian calculation tests
TEST_F(EgoMotionTest, LinearMotionJacobian__Correctness) {
    auto J = env::EgoMotion<float32>::calcLinearMotionJacobian(motion, dt);
    
    // Expected Jacobian values for linear motion
    const FloatType T = dt;
    const FloatType fac0 = math::pow<2>(T);
    const FloatType fac1 = static_cast<FloatType>(0.5) * fac0;
    
    const FloatType expected_dx_dv = T - fac1 * motion.w;
    const FloatType expected_dx_da = fac1;
    const FloatType expected_dx_dw = -fac0 * motion.v;
    
    const FloatType expected_dy_dv = fac1 * motion.w;
    const FloatType expected_dy_da = 0.0;
    const FloatType expected_dy_dw = fac1 * motion.v;
    
    // Verify Jacobian elements
    EXPECT_NEAR(J.at_unsafe(0, 0), expected_dx_dv, 1e-6);
    EXPECT_NEAR(J.at_unsafe(0, 1), expected_dx_da, 1e-6);
    EXPECT_NEAR(J.at_unsafe(0, 2), expected_dx_dw, 1e-6);
    EXPECT_NEAR(J.at_unsafe(1, 0), expected_dy_dv, 1e-6);
    EXPECT_NEAR(J.at_unsafe(1, 1), expected_dy_da, 1e-6);
    EXPECT_NEAR(J.at_unsafe(1, 2), expected_dy_dw, 1e-6);
    EXPECT_NEAR(J.at_unsafe(2, 0), 0.0, 1e-6);
    EXPECT_NEAR(J.at_unsafe(2, 1), 0.0, 1e-6);
    EXPECT_NEAR(J.at_unsafe(2, 2), T, 1e-6);
}
```

### Step 5: Documentation and Finalization

**Objective:** Provide complete documentation and examples for users.

**Tasks:**
- [ ] Update Doxygen documentation for new templatized interface
- [ ] Add usage examples for both covariance types
- [ ] Create migration guide for existing users
- [ ] Update README with new features
- [ ] Final validation of all test cases

**Documentation Examples:**

```cpp
// Example: Using UDU covariance for ego motion
auto egoMotionUDU = tracking::env::EgoMotion<
    tracking::math::CovarianceMatrixFactored, 
    float32
>{motionParams, geometry, dt};

// Example: Using full covariance for ego motion
auto egoMotionFull = tracking::env::EgoMotion<
    tracking::math::CovarianceMatrixFull, 
    float32
>{motionParams, geometry, dt};
```

## Risk Assessment and Mitigation

### Risks

1. **Numerical Instability**: Improper handling of positive definiteness
   - *Mitigation*: Comprehensive testing, assertions, epsilon clipping, baseline validation

2. **Template Complexity**: Increased compile times and complexity
   - *Mitigation*: Clear documentation, type constraints, examples, phased implementation

3. **Backward Compatibility**: Breaking existing code
   - *Mitigation*: Maintain full covariance support, provide migration guide, regression testing

4. **Performance Regression**: UDU implementation slower than expected
   - *Mitigation*: Performance testing, optimization if needed, baseline comparison

5. **Regression Issues**: New implementation produces different results
   - *Mitigation*: Comprehensive baseline testing, numerical equivalence validation

### Success Criteria

1. **Numerical Equivalence**: Refactored implementation produces identical results to baseline within numerical tolerance
2. **Positive Definiteness**: All covariance matrices remain positive definite throughout
3. **Performance**: Refactored implementation maintains or improves performance
4. **Type Safety**: Compile-time detection of covariance type mismatches
5. **Backward Compatibility**: Existing code continues to work without modification
6. **Test Coverage**: All baseline tests pass with refactored implementation

## Linear Workflow Benefits

This linear workflow ensures:

1. **Clear Progression**: Each step builds logically on the previous one
2. **Early Validation**: Baseline testing establishes expected behavior first
3. **Integrated Simplification**: Simplified UDU approach is part of the core architecture from the start
4. **Comprehensive Testing**: Validation at each step prevents regression
5. **Smooth Integration**: Motion models updated after core functionality is proven

## Resources and Timeline

- **Development Time**: ~8-10 weeks
- **Testing Resources**: Comprehensive test environment
- **Documentation**: Doxygen, examples, migration guide
- **Review**: Code reviews, mathematical validation

## Conclusion

This linear implementation plan provides a clear, step-by-step roadmap for implementing native UDU decomposition in ego motion calculations. By integrating the simplified UDU approach from the beginning and following a logical progression from testing to integration, the plan ensures numerical stability, maintainability, and backward compatibility while achieving all the original goals.