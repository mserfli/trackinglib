#ifndef D79C9F07_9851_46B8_9FAF_A055552247EE
#define D79C9F07_9851_46B8_9FAF_A055552247EE

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
  static void convertFrom(typename MM::StateVec& dstVec, const typename MM::StateVec& srcVec)
  {
    if (&srcVec != &dstVec)
    {
      dstVec = srcVec;
    }
  }
};

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateVecConverter<MotionModelCV<CovarianceMatrixType, FloatType>,
                        MotionModelCA<CovarianceMatrixType, FloatType>,
                        FloatType,
                        CovarianceMatrixType>
{
public:
  static void convertFrom(typename MotionModelCV<CovarianceMatrixType, FloatType>::StateVec&       dstVec,
                          const typename MotionModelCA<CovarianceMatrixType, FloatType>::StateVec& srcVec)
  {
    using DstType = MotionModelCV<CovarianceMatrixType, FloatType>;
    using SrcType = MotionModelCA<CovarianceMatrixType, FloatType>;

    dstVec[DstType::X]  = srcVec[SrcType::X];
    dstVec[DstType::VX] = srcVec[SrcType::VX];
    dstVec[DstType::Y]  = srcVec[SrcType::Y];
    dstVec[DstType::VY] = srcVec[SrcType::VY];
  }
};

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
class StateVecConverter<MotionModelCA<CovarianceMatrixType, FloatType>,
                        MotionModelCV<CovarianceMatrixType, FloatType>,
                        FloatType,
                        CovarianceMatrixType>
{
public:
  static void convertFrom(typename MotionModelCA<CovarianceMatrixType, FloatType>::StateVec&       dstVec,
                          const typename MotionModelCV<CovarianceMatrixType, FloatType>::StateVec& srcVec)
  {
    using DstType = MotionModelCA<CovarianceMatrixType, FloatType>;
    using SrcType = MotionModelCV<CovarianceMatrixType, FloatType>;

    dstVec[DstType::X]  = srcVec[SrcType::X];
    dstVec[DstType::VX] = srcVec[SrcType::VX];
    dstVec[DstType::AX] = static_cast<FloatType>(0.0);
    dstVec[DstType::Y]  = srcVec[SrcType::Y];
    dstVec[DstType::VY] = srcVec[SrcType::VY];
    dstVec[DstType::AY] = static_cast<FloatType>(0.0);
  }
};

} // namespace motion
} // namespace tracking

#endif // D79C9F07_9851_46B8_9FAF_A055552247EE
