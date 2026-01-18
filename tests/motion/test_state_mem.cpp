#include "trackingLib/motion/state_mem.hpp" // IWYU pragma: keep


// instatiate all templates for full coverage report
template class tracking::motion::StateMem<tracking::math::FullCovarianceMatrixPolicy<float32>, 4>;
template class tracking::motion::StateMem<tracking::math::FactoredCovarianceMatrixPolicy<float32>, 4>;
template class tracking::motion::StateMem<tracking::math::FullCovarianceMatrixPolicy<float32>, 6>;
template class tracking::motion::StateMem<tracking::math::FactoredCovarianceMatrixPolicy<float32>, 6>;

template class tracking::motion::StateMem<tracking::math::FullCovarianceMatrixPolicy<float64>, 4>;
template class tracking::motion::StateMem<tracking::math::FactoredCovarianceMatrixPolicy<float64>, 4>;
template class tracking::motion::StateMem<tracking::math::FullCovarianceMatrixPolicy<float64>, 6>;
template class tracking::motion::StateMem<tracking::math::FactoredCovarianceMatrixPolicy<float64>, 6>;
