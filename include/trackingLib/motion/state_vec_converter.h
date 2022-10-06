#ifndef D79C9F07_9851_46B8_9FAF_A055552247EE
#define D79C9F07_9851_46B8_9FAF_A055552247EE

#include "base/first_include.h"
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
class StateVecConverter;

template <typename MM, typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateVecConverter<MM, MM, FloatType, CovarianceMatrixType>
{
public:
  static void convertFrom(typename MM::StateVec& dstVec, const typename MM::StateVec& srcVec);
};

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateVecConverter<MotionModelCV<CovarianceMatrixType, FloatType>,
                        MotionModelCA<CovarianceMatrixType, FloatType>,
                        FloatType,
                        CovarianceMatrixType>
{
public:
  static void convertFrom(typename MotionModelCV<CovarianceMatrixType, FloatType>::StateVec&       dstVec,
                          const typename MotionModelCA<CovarianceMatrixType, FloatType>::StateVec& srcVec);
};

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateVecConverter<MotionModelCA<CovarianceMatrixType, FloatType>,
                        MotionModelCV<CovarianceMatrixType, FloatType>,
                        FloatType,
                        CovarianceMatrixType>
{
public:
  static void convertFrom(typename MotionModelCA<CovarianceMatrixType, FloatType>::StateVec&       dstVec,
                          const typename MotionModelCV<CovarianceMatrixType, FloatType>::StateVec& srcVec);
};

} // namespace motion
} // namespace tracking

#endif // D79C9F07_9851_46B8_9FAF_A055552247EE
