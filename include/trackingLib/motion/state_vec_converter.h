#ifndef D79C9F07_9851_46B8_9FAF_A055552247EE
#define D79C9F07_9851_46B8_9FAF_A055552247EE

#include "base/first_include.h" // IWYU pragma: keep
#include "motion/motion_model_ca.h"
#include "motion/motion_model_cv.h"

namespace tracking
{
namespace motion
{

template <typename MM_Dst, typename MM_Src>
class StateVecConverter;

template <typename MM>
class StateVecConverter<MM, MM>
{
public:
  static void convertFrom(typename MM::StateVec& dstVec, const typename MM::StateVec& srcVec);
};

template <typename CovarianceMatrixPolicy_>
class StateVecConverter<MotionModelCV<CovarianceMatrixPolicy_>, MotionModelCA<CovarianceMatrixPolicy_>>
{
public:
  static void convertFrom(typename MotionModelCV<CovarianceMatrixPolicy_>::StateVec&       dstVec,
                          const typename MotionModelCA<CovarianceMatrixPolicy_>::StateVec& srcVec);
};

template <typename CovarianceMatrixPolicy_>
class StateVecConverter<MotionModelCA<CovarianceMatrixPolicy_>, MotionModelCV<CovarianceMatrixPolicy_>>
{
public:
  static void convertFrom(typename MotionModelCA<CovarianceMatrixPolicy_>::StateVec&       dstVec,
                          const typename MotionModelCV<CovarianceMatrixPolicy_>::StateVec& srcVec);
};

} // namespace motion
} // namespace tracking

#endif // D79C9F07_9851_46B8_9FAF_A055552247EE
