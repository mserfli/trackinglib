```cpp
template <typename FloatType_>
class IObservationModel {
public:
  virtual ~IObservationModel() = default;
        
    // Filter-specific update methods
  virtual void update(const filter::KalmanFilter<FloatType>& filter) = 0;
  virtual void update(const filter::InformationFilter<FloatType>& filter) = 0;
};

template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType, typename FloatType_, sint32 Size_>
using ObservationMem = StateMem<CovarianceMatrixType, FloatType_, Size_>;

template <typename ObservationModel,
          template <typename FloatType, sint32 Size> class CovarianceMatrixType,
          typename FloatType,
          sint32 Size>
class ExtendedObservationModel
    : public IObservationModel<FloatType>
    , public ObservationMem<CovarianceMatrixType, FloatType, Size>
{
//... combines pure interface with storage class according to the observation dimension, not yet knowing about the semantics of the dimensions
};

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType>
class RangeBearingObservationModel 
    : public ExtendedObservationModel<RangeBearingObservationModel<CovarianceMatrixType, FloatType>, 
                                      CovarianceMatrixType, 
                                      FloatType, 
                                      2>
    , public generic::Update<RangeBearingObservationModel<CovarianceMatrixType, FloatType>, FloatType, CovarianceMatrixType>
{
  // knows about the dimension semantics, has CTOR to construct a measurement with z and R.
  // has a static method to calculate H matrix depending on the motion model (not yet sure whether the motion model is passed as a class or function template or even as a IMotionModel argument)
};

```
