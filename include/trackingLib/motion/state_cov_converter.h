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
          typename FloatType,
          template <typename FloatType_, sint32 Size_>
          class CovarianceMatrixType>
class StateCovConverter;

template <typename MM, typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateCovConverter<MM, MM, FloatType, CovarianceMatrixType>
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

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateCovConverter<MotionModelCV<CovarianceMatrixType, FloatType>,
                        MotionModelCA<CovarianceMatrixType, FloatType>,
                        FloatType,
                        CovarianceMatrixType>
{
public:
  static void convertFrom(typename MotionModelCV<CovarianceMatrixType, FloatType>::StateCov&       dstCov,
                          const typename MotionModelCA<CovarianceMatrixType, FloatType>::StateCov& srcCov)
  {
  }
};

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateCovConverter<MotionModelCA<CovarianceMatrixType, FloatType>,
                        MotionModelCV<CovarianceMatrixType, FloatType>,
                        FloatType,
                        CovarianceMatrixType>
{
public:
  static void convertFrom(typename MotionModelCA<CovarianceMatrixType, FloatType>::StateCov&       dstCov,
                          const typename MotionModelCV<CovarianceMatrixType, FloatType>::StateCov& srcCov)
  {
  }
};

} // namespace motion
} // namespace tracking

#endif // C29F4260_CDA5_4C57_8700_E478EBEC6295
