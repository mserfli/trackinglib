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
        +update(filter, observationModels...)
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

    class Update {
        +run(filter, motionModel, observationModels...)
    }

    class IObservationModel {
        +getDim()
    }

    class ExtendedObservationModel {
        +predictMeasurement(state)
        +computeJacobian(H, state)
        +computeInnovation(z, predicted)
    }

    class PositionObservationModel {
        +predictMeasurement(state)
        +computeJacobian(H, state)
    }

    class VelocityObservationModel {
        +predictMeasurement(state)
        +computeJacobian(H, state)
    }

    class RangeBearingObservationModel {
        +predictMeasurement(state)
        +computeJacobian(H, state)
        +computeInnovation(z, predicted)
    }

    class RangeBearingDopplerObservationModel {
        +predictMeasurement(state)
        +computeJacobian(H, state)
        +computeInnovation(z, predicted)
    }

    IMotionModel <|-- ExtendedMotionModel : extends
    ExtendedMotionModel <|-- MotionModelCV : extends
    ExtendedMotionModel <|-- MotionModelCA : extends
    ExtendedMotionModel *-- StateMem : has
    Predict --> KalmanFilter : uses
    Predict --> InformationFilter : uses
    ExtendedMotionModel --> Predict : uses for prediction
    Update --> KalmanFilter : uses
    Update --> InformationFilter : uses
    ExtendedMotionModel --> Update : uses for measurement update
    IObservationModel <|-- ExtendedObservationModel : extends
    ExtendedObservationModel <|-- PositionObservationModel : extends
    ExtendedObservationModel <|-- VelocityObservationModel : extends
    ExtendedObservationModel <|-- RangeBearingObservationModel : extends
    ExtendedObservationModel <|-- RangeBearingDopplerObservationModel : extends
    ExtendedObservationModel *-- StateMem : has (z, R)
    Update --> ExtendedObservationModel : evaluates
```</content>
<parameter name="filePath">doc/architecture.md