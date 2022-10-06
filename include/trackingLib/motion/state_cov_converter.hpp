#ifndef BA54EBB4_2356_43F9_BB94_0BC11475E511
#define BA54EBB4_2356_43F9_BB94_0BC11475E511

#include "motion/state_cov_converter.h"

namespace tracking
{
namespace motion
{

template <typename MM>
inline void StateCovConverter<MM, MM, typename MM::value_type>::convertFrom(typename MM::StateCov&       dstCov,
                                                                            const typename MM::StateCov& srcCov)
{
  if (&srcCov != &dstCov)
  {
    dstCov = srcCov;
  }
}

template <typename FloatType>
inline void StateCovConverter<
    MotionModelCV<math::CovarianceMatrixFull, FloatType>,
    MotionModelCA<math::CovarianceMatrixFull, FloatType>,
    FloatType>::convertFrom(typename MotionModelCV<math::CovarianceMatrixFull, FloatType>::StateCov&       dstCov,
                            const typename MotionModelCA<math::CovarianceMatrixFull, FloatType>::StateCov& srcCov)
{
  using DstType = MotionModelCV<math::CovarianceMatrixFull, FloatType>;
  using SrcType = MotionModelCA<math::CovarianceMatrixFull, FloatType>;

  static_assert(DstType::VX == DstType::X + 1);
  static_assert(DstType::VY == DstType::Y + 1);
  static_assert(SrcType::VX == SrcType::X + 1);
  static_assert(SrcType::VY == SrcType::Y + 1);
  // copy x,vx and its correlations
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::X,
                           SrcType::X,
                           DstType::X,
                           DstType::X>(srcCov);
  // copy cross correlations between x,vx and y,vy
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::X,
                           SrcType::Y,
                           DstType::X,
                           DstType::Y>(srcCov);
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::Y,
                           SrcType::X,
                           DstType::Y,
                           DstType::X>(srcCov);
  // copy y,vy and its correlations
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::Y,
                           SrcType::Y,
                           DstType::Y,
                           DstType::Y>(srcCov);
}

template <typename FloatType>
inline void StateCovConverter<
    MotionModelCV<math::CovarianceMatrixFactored, FloatType>,
    MotionModelCA<math::CovarianceMatrixFactored, FloatType>,
    FloatType>::convertFrom(typename MotionModelCV<math::CovarianceMatrixFactored, FloatType>::StateCov&       dstCov,
                            const typename MotionModelCA<math::CovarianceMatrixFactored, FloatType>::StateCov& srcCov)
{
  using DstType      = MotionModelCV<math::CovarianceMatrixFull, FloatType>;
  using SrcType      = MotionModelCA<math::CovarianceMatrixFull, FloatType>;
  constexpr auto one = static_cast<FloatType>(1.0);

  // create a permutation matrix from SrcType to DstType
  math::SquareMatrix<FloatType, SrcType::NUM_STATE_VARIABLES> A;
  A.setZeros();
  A(DstType::X, SrcType::X)   = one;
  A(DstType::VX, SrcType::VX) = one;
  A(DstType::Y, SrcType::Y)   = one;
  A(DstType::VY, SrcType::VY) = one;

  // fill dstCov with the resulting top left block
  dstCov.template fill<SrcType::NUM_STATE_VARIABLES, DstType::NUM_STATE_VARIABLES>(srcCov.apaT(A));
}


template <typename FloatType>
inline void StateCovConverter<
    MotionModelCA<math::CovarianceMatrixFull, FloatType>,
    MotionModelCV<math::CovarianceMatrixFull, FloatType>,
    FloatType>::convertFrom(typename MotionModelCA<math::CovarianceMatrixFull, FloatType>::StateCov&       dstCov,
                            const typename MotionModelCV<math::CovarianceMatrixFull, FloatType>::StateCov& srcCov)
{
  using DstType = MotionModelCA<math::CovarianceMatrixFull, FloatType>;
  using SrcType = MotionModelCV<math::CovarianceMatrixFull, FloatType>;

  static_assert(DstType::VX == DstType::X + 1);
  static_assert(DstType::VY == DstType::Y + 1);
  static_assert(SrcType::VX == SrcType::X + 1);
  static_assert(SrcType::VY == SrcType::Y + 1);
  // set ax,ay variance to 1.0
  dstCov.setIdentity();
  // copy x,vx and its correlations
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::X,
                           SrcType::X,
                           DstType::X,
                           DstType::X>(srcCov);
  // copy cross correlations between x,vx and y,vy
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::X,
                           SrcType::Y,
                           DstType::X,
                           DstType::Y>(srcCov);
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::Y,
                           SrcType::X,
                           DstType::Y,
                           DstType::X>(srcCov);
  // copy y,vy and its correlations
  dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                           SrcType::NUM_STATE_VARIABLES,
                           2,
                           2,
                           SrcType::Y,
                           SrcType::Y,
                           DstType::Y,
                           DstType::Y>(srcCov);
}

template <typename FloatType>
inline void StateCovConverter<
    MotionModelCA<math::CovarianceMatrixFactored, FloatType>,
    MotionModelCV<math::CovarianceMatrixFactored, FloatType>,
    FloatType>::convertFrom(typename MotionModelCA<math::CovarianceMatrixFactored, FloatType>::StateCov&       dstCov,
                            const typename MotionModelCV<math::CovarianceMatrixFactored, FloatType>::StateCov& srcCov)
{
  using DstType      = MotionModelCA<math::CovarianceMatrixFull, FloatType>;
  using SrcType      = MotionModelCV<math::CovarianceMatrixFull, FloatType>;
  constexpr auto one = static_cast<FloatType>(1.0);

  math::SquareMatrix<FloatType, DstType::NUM_STATE_VARIABLES> A;
  A.setZeros();
  A(DstType::X, SrcType::X)   = one;
  A(DstType::VX, SrcType::VX) = one;
  A(DstType::Y, SrcType::Y)   = one;
  A(DstType::VY, SrcType::VY) = one;

  dstCov.setIdentity();
  // copy CV into CA
  dstCov.template fill<SrcType::NUM_STATE_VARIABLES, SrcType::NUM_STATE_VARIABLES>(srcCov);
  // remap indeces by applying the permutation
  dstCov.apaT(A);
  // set ax,ay to variance 1.0
  dstCov.setDiagonal(DstType::AX, one);
  dstCov.setDiagonal(DstType::AY, one);
}

} // namespace motion
} // namespace tracking

#endif // BA54EBB4_2356_43F9_BB94_0BC11475E511
