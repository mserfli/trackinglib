#ifndef matrix_h
#define matrix_h

#include "base/atomic_types.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#include <Eigen/Dense>
#pragma clang diagnostic pop

namespace tracking
{
namespace base
{
template <typename FloatType, sint32 Rows, sint32 Cols>
using Matrix = Eigen::Matrix<FloatType, Rows, Cols>;

template <typename FloatType, sint32 Size>
using SquareMatrix = Eigen::Matrix<FloatType, Size, Size>;

template <typename FloatType, sint32 Size>
using DiagonalMatrix = Eigen::DiagonalMatrix<FloatType, Size>;

template <typename FloatType, sint32 Size>
using Vector = Eigen::Vector<FloatType, Size>;
} // namespace base
} // namespace tracking

#endif /* matrix_h */
