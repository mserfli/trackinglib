#include "gtest/gtest.h"
#include "trackingLib/math/linalg/covariance_matrix_factored.h"

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFull<float32, 3>;

// NOLINTEND(modernize-use-trailing-return-type)
