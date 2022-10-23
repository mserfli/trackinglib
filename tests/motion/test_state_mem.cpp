#include "gtest/gtest.h"
#include "trackingLib/motion/state_mem.hpp"

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFull, float32, 4>;
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFactored, float32, 4>;

// NOLINTEND(modernize-use-trailing-return-type)
