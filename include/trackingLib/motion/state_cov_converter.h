#ifndef C29F4260_CDA5_4C57_8700_E478EBEC6295
#define C29F4260_CDA5_4C57_8700_E478EBEC6295

#include "motion/motion_model_ca.h"
#include "motion/motion_model_cv.h"

namespace tracking
{
namespace motion
{

template <typename MM_Dst,
          typename MM_Src,
          typename FloatType>
class StateCovConverter;

template <typename MM>
class StateCovConverter<MM, MM, typename MM::value_type>
{
public:
  static void convertFrom(typename MM::StateCov& dstCov, const typename MM::StateCov& srcCov)
  {
    if (&srcCov != &dstCov)
    {
      dstCov = srcCov;
    }
  }
};

template <typename FloatType>
class StateCovConverter<MotionModelCV<math::CovarianceMatrixFull, FloatType>,
                        MotionModelCA<math::CovarianceMatrixFull, FloatType>, FloatType>
{
public:
  static void convertFrom(typename MotionModelCV<math::CovarianceMatrixFull, FloatType>::StateCov&       dstCov,
                          const typename MotionModelCA<math::CovarianceMatrixFull, FloatType>::StateCov& srcCov)
  {
  }
};

template <typename FloatType>
class StateCovConverter<MotionModelCV<math::CovarianceMatrixFactored, FloatType>,
                        MotionModelCA<math::CovarianceMatrixFactored, FloatType>, FloatType>
{
public:
  static void convertFrom(typename MotionModelCV<math::CovarianceMatrixFactored, FloatType>::StateCov&       dstCov,
                          const typename MotionModelCA<math::CovarianceMatrixFactored, FloatType>::StateCov& srcCov)
  {
  }
};

template <typename FloatType>
class StateCovConverter<MotionModelCA<math::CovarianceMatrixFull, FloatType>,
                        MotionModelCV<math::CovarianceMatrixFull, FloatType>, FloatType>
{
public:
  static void convertFrom(typename MotionModelCA<math::CovarianceMatrixFull, FloatType>::StateCov&       dstCov,
                          const typename MotionModelCV<math::CovarianceMatrixFull, FloatType>::StateCov& srcCov)
  {
  }
};

template <typename FloatType>
class StateCovConverter<MotionModelCA<math::CovarianceMatrixFactored, FloatType>,
                        MotionModelCV<math::CovarianceMatrixFactored, FloatType>, FloatType>
{
public:
  static void convertFrom(typename MotionModelCA<math::CovarianceMatrixFactored, FloatType>::StateCov&       dstCov,
                          const typename MotionModelCV<math::CovarianceMatrixFactored, FloatType>::StateCov& srcCov)
  {
  }
};

} // namespace motion
} // namespace tracking

#endif // C29F4260_CDA5_4C57_8700_E478EBEC6295
