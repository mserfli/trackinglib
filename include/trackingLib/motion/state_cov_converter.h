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

template <typename CovarianceMatricPolicy_>
class StateCovConverter<MotionModelCV<CovarianceMatricPolicy_>, MotionModelCA<CovarianceMatricPolicy_>>
{
public:
  static void convertFrom(typename MotionModelCV<CovarianceMatricPolicy_>::StateCov&       dstCov,
                          const typename MotionModelCA<CovarianceMatricPolicy_>::StateCov& srcCov);
};

template <typename CovarianceMatricPolicy_>
class StateCovConverter<MotionModelCA<CovarianceMatricPolicy_>, MotionModelCV<CovarianceMatricPolicy_>>
{
public:
  static void convertFrom(typename MotionModelCA<CovarianceMatricPolicy_>::StateCov&       dstCov,
                          const typename MotionModelCV<CovarianceMatricPolicy_>::StateCov& srcCov);
};

} // namespace motion
} // namespace tracking

#endif // C29F4260_CDA5_4C57_8700_E478EBEC6295
