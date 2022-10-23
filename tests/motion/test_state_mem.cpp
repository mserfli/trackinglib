#include "gtest/gtest.h"
#include "trackingLib/motion/state_mem.hpp"


// instatiate all templates for full coverage report
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFull, float32, 4>;
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFactored, float32, 4>;


