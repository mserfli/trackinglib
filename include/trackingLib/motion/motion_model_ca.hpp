#ifndef D2E08F36_A9FC_4D2E_9737_D64D5B314B9E
#define D2E08F36_A9FC_4D2E_9737_D64D5B314B9E

#include "motion/motion_model_ca.h"
#include "motion/state_cov_converter.h"
#include "motion/state_vec_converter.h"

namespace tracking
{
namespace motion
{

template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType, typename FloatType>
void MotionModelCA<CovarianceMatrixType, FloatType>::convertFrom(const MotionModelCV<CovarianceMatrixType, FloatType>& other)
{
  using other_type = MotionModelCV<CovarianceMatrixType, FloatType>;
  StateVecConverter<instance_type, other_type, FloatType, CovarianceMatrixType>::convertFrom(this->getVec(), other.getVec());
  StateCovConverter<instance_type, other_type, FloatType, CovarianceMatrixType>::convertFrom(this->getCov(), other.getCov());
}

}
}

#endif // D2E08F36_A9FC_4D2E_9737_D64D5B314B9E
