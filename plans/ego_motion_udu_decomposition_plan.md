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

## Implementation Phases

### Phase 0: Baseline Testing (NEW - CRITICAL FIRST STEP)

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

**Reference Values:**
- Document expected displacement and covariance values in test comments
- Include mathematical derivation references
- Store reference matrices for regression testing
- Add test cases covering the transition between linear and circular motion

**Files to Create:**
- `tests/motion/test_ego_motion.cpp` - Main test file
- `tests/motion/reference/ego_motion_reference_values.h` - Reference values header

**Success Criteria:**
- All baseline tests pass with current implementation
- Reference values documented and verified
- Test coverage includes linear, circular, and edge cases
- Covariance matrix properties validated (symmetry, positive definiteness)

### Phase 1: Infrastructure Preparation

**Tasks:**
- [ ] Create template parameter constraints for CovarianceMatrixType
- [ ] Add type traits to detect covariance matrix capabilities
- [ ] Update build system for new template instantiations
- [ ] Add comprehensive test infrastructure

**Files to Modify:**
- `include/trackingLib/env/ego_motion.h` - Add template parameters
- `include/trackingLib/env/ego_motion.hpp` - Update implementation
- `tests/motion/test_ego_motion.cpp` - Add test cases
- `CMakeLists.txt` - Add new test targets

### Phase 2: Templatized EgoMotion Class

**Tasks:**
- [ ] Convert EgoMotion to template class with CovarianceMatrixType parameter
- [ ] Update Displacement structure to use templatized covariance
- [ ] Implement type-specific displacement calculation methods
- [ ] Add SFINAE or concept constraints for template parameters

**Implementation Details:**

```cpp
// Updated EgoMotion class
template <template <typename, sint32> class CovarianceMatrixType, typename FloatType = float32>
class EgoMotion
{
public:
    using DisplacementCov = CovarianceMatrixType<FloatType, DS_NUM_VARIABLES>;
    
    struct Displacement
    {
        DisplacementVec vec{};
        DisplacementCov cov{DisplacementCov::Identity()};
        FloatType sinDeltaPsi{0.0};
        FloatType cosDeltaPsi{1.0};
    };
    
    // Constructor and existing interface
    // ...
    
    // Updated displacement calculation
    void calcDisplacement()
    {
        if (isLinearMotion())
        {
            calcLinearMotionDisplacement(_displacementCog, _motion, _dt);
        }
        else
        {
            calcCircularMotionDisplacement(_displacementCog, _motion, _dt);
        }
    }
};
```

### Phase 3: Refactored Implementation (Updated)

**Tasks:**
- [ ] Implement `calcLinearMotionJacobian()` for linear motion case
- [ ] Implement `calcCircularMotionJacobian()` for circular motion case
- [ ] Implement `calcDisplacementVector()` for common displacement calculation
- [ ] Implement `calcDisplacementCovariance()` with template specialization
- [ ] Add comprehensive unit tests for Jacobian methods
- [ ] Validate numerical equivalence between approaches
- [ ] Ensure proper positive definite initialization
- [ ] Add numerical stability checks and assertions

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
    
    // Common covariance calculation (templatized)
    void calcDisplacementCovariance(Displacement& displacement,
                                  const math::SquareMatrix<FloatType, 3, true>& J,
                                  const InertialMotion& motion);
};
```

**Benefits of Refactored Design:**
- Separates Jacobian calculation from covariance computation
- Common logic for displacement vector calculation
- Pure function Jacobian methods for better testability
- Template specialization for covariance type handling
- Reduced code duplication

**Correct Implementation Pattern:**

```cpp
template <typename FloatType_>
void calcLinearMotionDisplacementUDU(
    CovarianceMatrixFactored<FloatType_, 3>& cov, 
    const InertialMotion& motion, 
    FloatType_ dt)
{
    // 1. Build Jacobian matrix J
    using JacobianMatrix = math::SquareMatrix<FloatType_, 3, true>;
    JacobianMatrix J{};
    // ... set Jacobian elements ...
    
    // 2. Build positive definite diagonal input covariance Pin
    DiagonalMatrix<FloatType_, 3> Pin{};
    Pin.at_unsafe(0) = math::pow<2>(motion.sv); // σ_v² > 0
    Pin.at_unsafe(1) = math::pow<2>(motion.sa); // σ_a² > 0
    Pin.at_unsafe(2) = math::pow<2>(motion.sw); // σ_ω² > 0
    
    // 3. Initialize U and D with proper positive definite values
    TriangularMatrix<FloatType_, 3, false, true> U = TriangularMatrix<FloatType_, 3, false, true>::Identity();
    DiagonalMatrix<FloatType_, 3> D{Pin}; // Start with positive definite D
    
    // 4. Apply Modified Gram-Schmidt to compute UDU factorization
    // This computes: UDU' = J * (U*D*U') * J^T = (J*U) * D * (J*U)'
    math::ModifiedGramSchmidt<FloatType_, 3>::run(U, D, J);
    
    // 5. Create the result
    cov = CovarianceMatrixFactored<FloatType_, 3>{std::move(U), std::move(D)};
}
```

**Key Numerical Considerations:**
- Always initialize D with positive values (never zeros)
- Ensure Jacobian is full rank for valid motion parameters
- Add assertions to verify positive definiteness
- Use epsilon clipping to prevent numerical underflow

### Phase 4: Integration Testing

**Tasks:**
- [ ] Add regression tests comparing old vs new implementation
- [ ] Test numerical equivalence between refactored and original code
- [ ] Validate Jacobian calculation methods independently
- [ ] Test displacement consistency between old and new implementations
- [ ] Test covariance equivalence between both approaches
- [ ] Add performance benchmarking tests

**Additional Test Cases:**
- **Jacobian Validation**: Independent testing of Jacobian calculation methods
- **Displacement Consistency**: Verify displacement vectors match between old and new implementations
- **Covariance Equivalence**: Ensure both covariance implementations produce equivalent results
- **Numerical Stability**: Test edge cases for both Jacobian types
- **Regression Testing**: Compare results with baseline tests from Phase 0

**Test Structure:**

```cpp
// Regression test comparing old and new implementations
TEST_F(EgoMotionTest, RefactoredImplementation__NumericalEquivalence) {
    // Test with current implementation
    auto egoMotionOld = env::EgoMotion<float32>{motion, geometry, dt};
    auto oldDisplacement = egoMotionOld.getDisplacementCog();
    
    // Test with refactored implementation (after it's implemented)
    // This will be added during Phase 3
    // auto egoMotionNew = env::EgoMotionRefactored<float32>{motion, geometry, dt};
    // auto newDisplacement = egoMotionNew.getDisplacementCog();
    
    // Verify displacement vectors match
    // EXPECT_NEAR(oldDisplacement.vec.at_unsafe(0), newDisplacement.vec.at_unsafe(0), 1e-6);
    // EXPECT_NEAR(oldDisplacement.vec.at_unsafe(1), newDisplacement.vec.at_unsafe(1), 1e-6);
    // EXPECT_NEAR(oldDisplacement.vec.at_unsafe(2), newDisplacement.vec.at_unsafe(2), 1e-6);
    
    // Verify covariance matrices match
    // auto oldCov = oldDisplacement.cov();
    // auto newCov = newDisplacement.cov();
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j < 3; ++j) {
    //         EXPECT_NEAR(oldCov.at_unsafe(i, j), newCov.at_unsafe(i, j), 1e-6);
    //     }
    // }
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

### Phase 5: Full Matrix Implementation (Backward Compatibility)

**Tasks:**
- [ ] Implement `calcLinearMotionDisplacementFull()` using existing approach
- [ ] Implement `calcCircularMotionDisplacementFull()` using existing approach
- [ ] Ensure API compatibility with existing code

**Implementation:**

```cpp
template <typename FloatType_>
void calcLinearMotionDisplacementFull(
    CovarianceMatrixFull<FloatType_, 3>& cov, 
    const InertialMotion& motion, 
    FloatType_ dt)
{
    // Existing implementation using apaT
    DisplacementCov tempCov{};
    tempCov.setZeros();
    
    // Set diagonal elements
    tempCov.at_unsafe(0, 0) = math::pow<2>(motion.sv);
    tempCov.at_unsafe(1, 1) = math::pow<2>(motion.sa);
    tempCov.at_unsafe(2, 2) = math::pow<2>(motion.sw);
    
    // Build Jacobian
    using JacobianMatrix = math::SquareMatrix<FloatType_, 3, true>;
    JacobianMatrix J{};
    // ... set Jacobian elements ...
    
    // Compute covariance
    tempCov.apaT(J);
    cov = std::move(tempCov);
}
```

### Phase 5: Motion Model Integration

**Tasks:**
- [ ] Update MotionModelCV to use templatized EgoMotion
- [ ] Update MotionModelCA to use templatized EgoMotion
- [ ] Ensure type consistency between motion models and ego motion
- [ ] Update prediction methods to handle both covariance types

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

### Phase 6: Testing and Validation

**Tasks:**
- [ ] Create comprehensive test suite for UDU ego motion
- [ ] Test numerical equivalence between full and factored implementations
- [ ] Test edge cases (small velocities, zero acceleration, etc.)
- [ ] Test numerical stability with extreme parameters
- [ ] Performance benchmarking

**Test Cases Required:**
1. **Numerical Equivalence**: Verify UDU and full implementations produce equivalent results
2. **Positive Definiteness**: Ensure all covariance matrices remain positive definite
3. **Edge Cases**: Small ω, zero acceleration, high velocities
4. **Numerical Stability**: Ill-conditioned parameters, extreme values
5. **Performance**: Compare computation time between approaches

### Phase 7: Documentation and Examples

**Tasks:**
- [ ] Update Doxygen documentation for new templatized interface
- [ ] Add usage examples for both covariance types
- [ ] Create migration guide for existing users
- [ ] Update README with new features

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

## Updated Risk Assessment and Mitigation

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

## Updated Timeline and Milestones

### Phase 0: Baseline Testing (Week 1-2)
- Create comprehensive test suite for current implementation
- Document expected results and reference values
- Validate all baseline tests pass
- Establish regression testing framework

### Phase 1: Refactored Architecture (Week 3-4)
- Implement separated Jacobian calculation methods
- Create common displacement vector logic
- Implement templatized covariance calculation
- Maintain API compatibility

### Phase 2: UDU Implementation (Week 5-6)
- Implement UDU-specific covariance calculation
- Ensure numerical stability with proper initialization
- Add comprehensive assertions and checks
- Validate mathematical equivalence

### Phase 3: Integration and Testing (Week 7-8)
- Update motion models to use refactored EgoMotion
- Add regression tests comparing implementations
- Test numerical equivalence comprehensively
- Validate system-wide integration

### Phase 4: Documentation and Finalization (Week 9-10)
- Document refactored interface clearly
- Update examples for both covariance types
- Create migration guide for users
- Final validation of all test cases

## Dependencies

- Modified Gram-Schmidt implementation (existing)
- CovarianceMatrixFactored class (existing)
- CovarianceMatrixFull class (existing)
- Motion model infrastructure (existing)

## Resources Required

- Development time: ~8-10 weeks
- Testing resources: Comprehensive test environment
- Documentation: Doxygen, examples, migration guide
- Review: Code reviews, mathematical validation

## Conclusion

This plan provides a comprehensive roadmap for implementing native UDU decomposition in ego motion calculations while maintaining numerical stability, backward compatibility, and type safety. The phased approach ensures careful validation at each stage and addresses the key concern about positive definiteness in UDU factorization.