# Architecture Overview

## Class Diagram

```mermaid
classDiagram
    class IMotionModel {
        +predict(dt, filter, egoMotion)
        +getState()
        +getCovariance()
    }

    class ExtendedMotionModel {
        +predict(dt, filter, egoMotion)
        +update(observationModel, filter, mode)
    }

    class MotionModelCV {
        +predict(dt, filter, egoMotion)
        +computeA(A, dt)
        +computeQ(Q, dt)
        +computeG(G, dt)
    }

    class MotionModelCA {
        +predict(dt, filter, egoMotion)
        +computeA(A, dt)
        +computeQ(Q, dt)
        +computeG(G, dt)
    }

    class StateMem {
        -StateVec state_
        -CovarianceType covariance_
        +getState()
        +getCovariance()
    }

    class KalmanFilter {
        +predictCovariance(P, A, G, Q)
        +updateState(x, P, z, H, R)
    }

    class InformationFilter {
        +predictCovariance(Y, A, G, Q)
        +updateState(y, Y, z, H, R)
    }

    class Predict {
        +run(dt, filter, egoMotion)
    }

    IMotionModel <|-- ExtendedMotionModel : extends
    ExtendedMotionModel <|-- MotionModelCV : extends
    ExtendedMotionModel <|-- MotionModelCA : extends
    ExtendedMotionModel *-- StateMem : has
    Predict --> KalmanFilter : uses
    Predict --> InformationFilter : uses
    ExtendedMotionModel --> Predict : uses for prediction
```</content>
<parameter name="filePath">doc/architecture.md