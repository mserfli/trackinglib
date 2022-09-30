#ifndef CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
#define CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5

#include "base/first_include.h"
#include "env/ego_motion.h"
#include "filter/information_filter.h"
#include "filter/kalman_filter.h"
#include "motion/generic_predict_common.h"
#include "motion/state_mem.h"


namespace tracking
{
namespace motion
{
namespace generic
{

/// \brief Base class to extend any motion model with a generic prediction functionality using the CRTP pattern
/// \tparam MotionModel           The underlying MotionModel
/// \tparam FloatType             The float type representation
/// \tparam CovarianceMatrixType  The used covariance matrix type
template <typename MotionModel, typename FloatType, template <typename FloatType_, sint32 Size> class CovarianceMatrixType>
class Predict
{
};

/// \brief Partial specialization of the generic predictor for the basic square covariance matrix
/// \tparam MotionModel           The underlying MotionModel
/// \tparam FloatType             The float type representation
template <typename MotionModel, typename FloatType>
class Predict<MotionModel, FloatType, math::CovarianceMatrixFull>: public PredictCommon<MotionModel, FloatType>
{
public:
  /// \brief State prediction with ego motion compensation using a KalmanFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const FloatType dt, const filter::KalmanFilter<FloatType>& filter, const env::EgoMotion<FloatType>& egoMotion)
  {
    static typename PredictCommon<MotionModel, FloatType>::Storage data{};
    PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

    auto& underlying = static_cast<MotionModel&>(*this);
    // apply ego motion compensation on P
    auto P = make_unique<typename MotionModel::StateCov>((data.Go * underlying.getCov() * data.Go.transpose()) +
                                                         (data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose()));

    filter.predictCovariance(*P, data.A, data.G, data.Q);

    underlying.setCov(std::move(P));
  }

  /// \brief State prediction with ego motion compensation using an InformationFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const FloatType dt, const filter::InformationFilter<FloatType>& filter, const env::EgoMotion<FloatType>& egoMotion)
  {
    static typename PredictCommon<MotionModel, FloatType>::Storage data{};
    PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

    auto& underlying = static_cast<MotionModel&>(*this);
    assert(underlying.getCov().isInverse() && "covariance has to represent the inverse covariance");
#if 0    
    // TODO(matthias): ego motion compensation is quite complicated here, maybe neglect influence of Ge*Pe*Ge'
    // reconstruct P which might cause issues because P becomes extremly large
    static auto postP = underlying.getCov().inverse();
    // apply ego motion compensation on P
    static auto compP = (data.Go * postP * data.Go.transpose())
                      + (data.Ge * egoMotion.getDisplacementCog().cov * data.Ge.transpose());
    // calc compensated Y
    static auto Y = make_unique<typename MotionModel::StateCov>(compP.inverse(), true);
#else
    static auto invGo = data.Go.inverse();
    // calc compensated Y, neglecting Ge
    auto Y = make_unique<typename MotionModel::StateCov>(invGo.transpose() * underlying.getCov() * invGo, true);
#endif
    filter.predictCovariance(*Y, data.A, data.G, data.Q);

    underlying.setCov(std::move(Y));
  };
};

/// \brief Partial specialization of the generic predictor for a factored covariance matrix
/// \tparam MotionModel           The underlying MotionModel
/// \tparam FloatType             The float type representation
template <typename MotionModel, typename FloatType>
class Predict<MotionModel, FloatType, math::CovarianceMatrixFactored>: public PredictCommon<MotionModel, FloatType>
{
public:
  /// \brief State prediction with ego motion compensation using a KalmanFilter
  /// \param[in] dt         The delta time from last state to predicted state
  /// \param[in] filter     The filter instance
  /// \param[in] egoMotion  The known egoMotion from last state to predicted state
  void run(const FloatType dt, const filter::KalmanFilter<FloatType>& filter, const env::EgoMotion<FloatType>& egoMotion)
  {
    static typename PredictCommon<MotionModel, FloatType>::Storage data{};
    PredictCommon<MotionModel, FloatType>::run(data, dt, egoMotion);

    auto& underlying = static_cast<MotionModel&>(*this);

    static typename MotionModel::StateMatrix AGo = data.A * data.Go;

    // TODO(matthias): complete calculations of the factored predict

    // TODO(matthias): Optimization - provide a new BlockDiagonal matrix class to reduce operations on known zero elements
    static typename MotionModel::AugmentedProcessNoiseDiagMatrix Qstar; // [De 0; 0 Q]
    // TODO(matthias): Optimization - we could also have a vector of matrices to avoid constructing augmented matrices with copy
    // operations
    static typename MotionModel::AugmentedProcessNoiseMappingMatrix Gstar; // [A*Ge*Ue G]

    auto P = make_unique<typename MotionModel::StateCov>(underlying.getCov());

    filter.predictCovariance(*P, AGo, Gstar, Qstar);

    underlying.setCov(std::move(P));
  }
};

} // namespace generic
} // namespace motion
} // namespace tracking

#endif // CFE4ADAC_CBD6_4488_B120_96D9FBE6C1A5
