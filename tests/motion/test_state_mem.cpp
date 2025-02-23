#include "trackingLib/motion/state_mem.hpp" // IWYU pragma: keep


// instatiate all templates for full coverage report
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFull, float32, 4>;
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFactored, float32, 4>;
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFull, float32, 6>;
template class tracking::motion::StateMem<tracking::math::CovarianceMatrixFactored, float32, 6>;
