#ifndef AE0B3445_6354_4B13_BF8F_59880FFFC470
#define AE0B3445_6354_4B13_BF8F_59880FFFC470

#include "motion/motion_model_cv.h"
#include "motion/state_cov_converter.h"
#include "motion/state_vec_converter.h"

template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType, typename FloatType>
void tracking::motion::MotionModelCV<CovarianceMatrixType, FloatType>::convertFrom(const MotionModelCA<CovarianceMatrixType, FloatType>& other)
{
  using other_type = MotionModelCA<CovarianceMatrixType, FloatType>;
  StateVecConverter<instance_type, other_type, FloatType, CovarianceMatrixType>::convertFrom(this->getVec(), other.getVec());
  StateCovConverter<instance_type, other_type, FloatType>::convertFrom(this->getCov(), other.getCov());
}


#endif // AE0B3445_6354_4B13_BF8F_59880FFFC470
