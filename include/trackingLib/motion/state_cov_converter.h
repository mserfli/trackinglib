#ifndef C29F4260_CDA5_4C57_8700_E478EBEC6295
#define C29F4260_CDA5_4C57_8700_E478EBEC6295

#include "base/first_include.h" // IWYU pragma: keep
#include "motion/motion_model_ca.h"
#include "motion/motion_model_cv.h"

namespace tracking
{
namespace motion
{

template <typename MM_Dst, typename MM_Src>
class StateCovConverter;

template <typename MM>
class StateCovConverter<MM, MM>
{
public:
  static void convertFrom(typename MM::StateCov& dstCov, const typename MM::StateCov& srcCov);
};

template <typename CovarianceMatrixPolicy_>
class StateCovConverter<MotionModelCV<CovarianceMatrixPolicy_>, MotionModelCA<CovarianceMatrixPolicy_>>
{
public:
  static void convertFrom(typename MotionModelCV<CovarianceMatrixPolicy_>::StateCov&       dstCov,
                          const typename MotionModelCA<CovarianceMatrixPolicy_>::StateCov& srcCov);
};

template <typename CovarianceMatrixPolicy_>
class StateCovConverter<MotionModelCA<CovarianceMatrixPolicy_>, MotionModelCV<CovarianceMatrixPolicy_>>
{
public:
  static void convertFrom(typename MotionModelCA<CovarianceMatrixPolicy_>::StateCov&       dstCov,
                          const typename MotionModelCV<CovarianceMatrixPolicy_>::StateCov& srcCov);
};

} // namespace motion
} // namespace tracking

#endif // C29F4260_CDA5_4C57_8700_E478EBEC6295
