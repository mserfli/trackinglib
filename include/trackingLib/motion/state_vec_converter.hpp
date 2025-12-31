#ifndef DE5D4081_1ED8_4BFB_B9C5_32EE6E10ADFC
#define DE5D4081_1ED8_4BFB_B9C5_32EE6E10ADFC

#include "motion/state_vec_converter.h"

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
inline void StateVecConverter<MM, MM, FloatType, CovarianceMatrixType>::convertFrom(typename MM::StateVec&       dstVec,
                                                                                    const typename MM::StateVec& srcVec)
{
  if (&srcVec != &dstVec)
  {
    dstVec = srcVec;
  }
}

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
inline void StateVecConverter<
    MotionModelCV<CovarianceMatrixType, FloatType>,
    MotionModelCA<CovarianceMatrixType, FloatType>,
    FloatType,
    CovarianceMatrixType>::convertFrom(typename MotionModelCV<CovarianceMatrixType, FloatType>::StateVec&       dstVec,
                                       const typename MotionModelCA<CovarianceMatrixType, FloatType>::StateVec& srcVec)
{
  using DstType = MotionModelCV<CovarianceMatrixType, FloatType>;
  using SrcType = MotionModelCA<CovarianceMatrixType, FloatType>;

  dstVec.at_unsafe(DstType::X)  = srcVec.at_unsafe(SrcType::X);
  dstVec.at_unsafe(DstType::VX) = srcVec.at_unsafe(SrcType::VX);
  dstVec.at_unsafe(DstType::Y)  = srcVec.at_unsafe(SrcType::Y);
  dstVec.at_unsafe(DstType::VY) = srcVec.at_unsafe(SrcType::VY);
}

template <typename FloatType, template <typename FloatType_, sint32 Size_> class CovarianceMatrixType>
inline void StateVecConverter<
    MotionModelCA<CovarianceMatrixType, FloatType>,
    MotionModelCV<CovarianceMatrixType, FloatType>,
    FloatType,
    CovarianceMatrixType>::convertFrom(typename MotionModelCA<CovarianceMatrixType, FloatType>::StateVec&       dstVec,
                                       const typename MotionModelCV<CovarianceMatrixType, FloatType>::StateVec& srcVec)
{
  using DstType = MotionModelCA<CovarianceMatrixType, FloatType>;
  using SrcType = MotionModelCV<CovarianceMatrixType, FloatType>;

  dstVec.at_unsafe(DstType::X)  = srcVec.at_unsafe(SrcType::X);
  dstVec.at_unsafe(DstType::VX) = srcVec.at_unsafe(SrcType::VX);
  dstVec.at_unsafe(DstType::AX) = static_cast<FloatType>(0.0);
  dstVec.at_unsafe(DstType::Y)  = srcVec.at_unsafe(SrcType::Y);
  dstVec.at_unsafe(DstType::VY) = srcVec.at_unsafe(SrcType::VY);
  dstVec.at_unsafe(DstType::AY) = static_cast<FloatType>(0.0);
}

} // namespace motion
} // namespace tracking

#endif // DE5D4081_1ED8_4BFB_B9C5_32EE6E10ADFC
