# Filter Prediction Flow for Kalman and Information Filter

## Overview

This document shows the complete prediction flow for both Kalman Filter and Information Filter, highlighting the differences in state representation and covariance handling.

## Prediction Flow Diagram

```mermaid
sequenceDiagram
    participant User
    participant MM as MotionModelCV
    participant GP as GenericPredict::run
    participant GPC as PredictCommon::run
    participant KF as KalmanFilter
    participant IF as InformationFilter
    
    User->>MM: predict(dt, filter, egoMotion)
    MM->>GP: run(dt, filter, egoMotion)
    
    Note over GP: Create static data storage
    
    rect rgb(230, 255, 230)
        Note over GP: State Space Transformation
        alt InformationFilter
            GP->>GP: convertStateVecIntoStateSpace()
            Note right of GP: x = Y^-1 * y<br/>Transform state vector to<br/>state space
        end
    end
    
    GP->>GPC: run(data, dt, egoMotion)
    
    rect rgb(240, 248, 255)
        Note over GPC,MM: State Prediction Phase - Same for both filters
        GPC->>MM: computeEgoMotionCompensationMatrices(Ge, Go, egoMotion)
        MM-->>GPC: Ge, Go matrices
        
        alt ego displacement != 0
            GPC->>MM: compensateState(egoMotion)
            Note right of MM: x = g(x, ego_disp)<br/>Exact nonlinear compensation
            MM-->>GPC: State x compensated
        end
        
        GPC->>MM: computeA(A, dt)
        MM-->>GPC: A matrix
        
        GPC->>MM: applyProcessModel(dt)
        Note right of MM: x_k+1 = f(x_k, dt)<br/>Exact nonlinear transition
        MM-->>GPC: State x predicted
    end
    
    rect rgb(255, 250, 240)
        Note over GPC,MM: Process Noise Phase - Same for both filters
        GPC->>MM: computeQ(Q, dt)
        MM-->>GPC: Q diagonal matrix
        GPC->>MM: computeG(G, dt)
        MM-->>GPC: G matrix
    end
    
    GPC-->>GP: data populated
    
    rect rgb(240, 255, 240)
        Note over GP,IF: Covariance Prediction Phase
        alt KalmanFilter with Factored Covariance
            GP->>GP: Get P from motion model
            alt ego displacement == 0
                GP->>KF: predictCovariance(P, AGo, G, Q)
            else ego displacement != 0
                GP->>KF: predictCovariance(P, AGo, Gstar, Qstar)
            end
            Note right of KF: Thornton update<br/>P = AGo*P*AGo' + G*Q*G'
            KF-->>GP: P_k+1 updated
        else KalmanFilter with Full Covariance
            GP->>GP: Get P from motion model
            alt ego displacement != 0
                GP->>GP: P = Go*P*Go' + Ge*Pe*Ge'
                GP->>GP: P.symmetrize()
            end
            GP->>KF: predictCovariance(P, A, G, Q)
            Note right of KF: P = A*P*A' + G*Q*G'
            KF-->>GP: P_k+1 updated
        else InformationFilter with Factored Covariance
            GP->>GP: Get Y from motion model
            alt ego displacement == 0
                GP->>IF: predictCovariance(Y, AGo, G, Q)
            else ego displacement != 0
                GP->>IF: predictCovariance(Y, AGo, Gstar, Qstar)
            end
            Note right of IF: Information form update<br/>Y_k+1 from UDU factors
            IF-->>GP: Y_k+1 updated
        else InformationFilter with Full Covariance
            GP->>GP: Get Y from motion model
            alt ego displacement == 0
                GP->>IF: predictCovariance(Y, A, G, Q)
                Note right of IF: Y_k+1 = inv(A*Y^-1*A' + G*Q*G')
            else ego displacement != 0
                Note right of GP: Woodbury identity approach
                GP->>GP: Ytr = Go'^-1 * Y * Go^-1
                GP->>GP: Y = Ytr - Ytr*Ge * inv(Pe^-1 + Ge'*Ytr*Ge)^-1 * Ge'*Ytr
                GP->>IF: predictCovariance(Ytr, A, G, Q)
            end
            IF-->>GP: Y_k+1 updated
        end
    end
    
    rect rgb(230, 255, 230)
        Note over GP: State Space Transformation
        alt InformationFilter
            GP->>GP: convertStateVecIntoInformationSpace()
            Note right of GP: y_k+1 = Y_k+1 * x_k+1<br/>Transform state vector to<br/>information space
        end
    end
    
    GP-->>MM: Prediction complete
    MM-->>User: Kalman: x_k+1, P_k+1<br/>Information: y_k+1, Y_k+1
```

## Key Differences Between Filters

| Aspect | Kalman Filter | Information Filter |
|--------|---------------|-------------------|
| **State Vector** | x (state) | y = Y * x (information vector) |
| **Covariance** | P (covariance) | Y = P^-1 (information matrix) |
| **Pre-Processing** | None | `convertStateVecIntoStateSpace()` |
| **Post-Processing** | None | `convertStateVecIntoInformationSpace()` |
| **Covariance Update** | P = A*P*A' + G*Q*G' | Y = inv(A*Y^-1*A' + G*Q*G') |

## State Representation

### Kalman Filter
- State vector: `x` in state space
- Covariance: `P` (covariance matrix)
- No transformation needed

### Information Filter
- State vector: `y = Y * x` (information vector)
- Covariance: `Y = P^-1` (information matrix)
- Requires transformation before/after state prediction

## Mathematical Formulation

### State Prediction (Same for both filters)
```
x_compensated = g(x, egoMotion)     // Nonlinear ego motion compensation
x_k+1 = f(x_compensated, dt)        // Nonlinear state transition
```

### Covariance Prediction

**Kalman Filter:**
```
P_k+1 = A * P_k * A' + G * Q * G'
```

**Information Filter:**
```
Y_k+1 = inv(A * Y_k^-1 * A' + G * Q * G')
```

Using Woodbury identity for numerical stability:
```
Y_k+1 = Y_k - Y_k * A' * inv(A * Y_k^-1 * A' + G*Q*G') * A * Y_k
```
