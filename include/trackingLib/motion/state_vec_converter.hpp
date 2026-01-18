#ifndef DE5D4081_1ED8_4BFB_B9C5_32EE6E10ADFC
#define DE5D4081_1ED8_4BFB_B9C5_32EE6E10ADFC

#include "motion/state_vec_converter.h"

namespace tracking
{
namespace motion
{

template <typename MM>
inline void StateVecConverter<MM, MM>::convertFrom(typename MM::StateVec& dstVec, const typename MM::StateVec& srcVec)
{
  if (&srcVec != &dstVec)
  {
    dstVec = srcVec;
  }
}

template <typename CovarianceMatrixPolicy_>
inline void StateVecConverter<MotionModelCV<CovarianceMatrixPolicy_>, MotionModelCA<CovarianceMatrixPolicy_>>::convertFrom(
    typename MotionModelCV<CovarianceMatrixPolicy_>::StateVec&       dstVec,
    const typename MotionModelCA<CovarianceMatrixPolicy_>::StateVec& srcVec)
{
  using DstType = MotionModelCV<CovarianceMatrixPolicy_>;
  using SrcType = MotionModelCA<CovarianceMatrixPolicy_>;

  dstVec.at_unsafe(DstType::X)  = srcVec.at_unsafe(SrcType::X);
  dstVec.at_unsafe(DstType::VX) = srcVec.at_unsafe(SrcType::VX);
  dstVec.at_unsafe(DstType::Y)  = srcVec.at_unsafe(SrcType::Y);
  dstVec.at_unsafe(DstType::VY) = srcVec.at_unsafe(SrcType::VY);
}

template <typename CovarianceMatrixPolicy_>
inline void StateVecConverter<MotionModelCA<CovarianceMatrixPolicy_>, MotionModelCV<CovarianceMatrixPolicy_>>::convertFrom(
    typename MotionModelCA<CovarianceMatrixPolicy_>::StateVec&       dstVec,
    const typename MotionModelCV<CovarianceMatrixPolicy_>::StateVec& srcVec)
{
  using DstType   = MotionModelCA<CovarianceMatrixPolicy_>;
  using SrcType   = MotionModelCV<CovarianceMatrixPolicy_>;
  using FloatType = typename CovarianceMatrixPolicy_::FloatType;

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
