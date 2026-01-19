#ifndef BA54EBB4_2356_43F9_BB94_0BC11475E511
#define BA54EBB4_2356_43F9_BB94_0BC11475E511

#include "motion/state_cov_converter.h"

namespace tracking
{
namespace motion
{

template <typename MM>
inline void StateCovConverter<MM, MM>::convertFrom(typename MM::StateCov& dstCov, const typename MM::StateCov& srcCov)
{
  if (&srcCov != &dstCov)
  {
    dstCov = srcCov;
  }
}

template <typename CovarianceMatricPolicy_>
inline void StateCovConverter<MotionModelCV<CovarianceMatricPolicy_>, MotionModelCA<CovarianceMatricPolicy_>>::convertFrom(
    typename MotionModelCV<CovarianceMatricPolicy_>::StateCov&       dstCov,
    const typename MotionModelCA<CovarianceMatricPolicy_>::StateCov& srcCov)
{
  using DstType = MotionModelCV<CovarianceMatricPolicy_>;
  using SrcType = MotionModelCA<CovarianceMatricPolicy_>;

  if constexpr (CovarianceMatricPolicy_::is_factored)
  {
    using FloatType    = typename CovarianceMatricPolicy_::FloatType;
    constexpr auto one = static_cast<FloatType>(1.0);

    // create a permutation matrix from SrcType to DstType
    math::SquareMatrix<FloatType, SrcType::NUM_STATE_VARIABLES> A;
    A.setZeros();
    A.at_unsafe(DstType::X, SrcType::X)   = one;
    A.at_unsafe(DstType::VX, SrcType::VX) = one;
    A.at_unsafe(DstType::Y, SrcType::Y)   = one;
    A.at_unsafe(DstType::VY, SrcType::VY) = one;

    // fill dstCov with the resulting top left block
    dstCov.template fill<SrcType::NUM_STATE_VARIABLES, DstType::NUM_STATE_VARIABLES>(srcCov.apaT(A));
  }
  else
  {
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
                             SrcType::StateCov::IsRowMajor,
                             DstType::X,
                             DstType::X>(srcCov);
    // copy cross correlations between x,vx and y,vy
    dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                             SrcType::NUM_STATE_VARIABLES,
                             2,
                             2,
                             SrcType::X,
                             SrcType::Y,
                             SrcType::StateCov::IsRowMajor,
                             DstType::X,
                             DstType::Y>(srcCov);
    dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                             SrcType::NUM_STATE_VARIABLES,
                             2,
                             2,
                             SrcType::Y,
                             SrcType::X,
                             SrcType::StateCov::IsRowMajor,
                             DstType::Y,
                             DstType::X>(srcCov);
    // copy y,vy and its correlations
    dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                             SrcType::NUM_STATE_VARIABLES,
                             2,
                             2,
                             SrcType::Y,
                             SrcType::Y,
                             SrcType::StateCov::IsRowMajor,
                             DstType::Y,
                             DstType::Y>(srcCov);
  }
}

template <typename CovarianceMatricPolicy_>
inline void StateCovConverter<MotionModelCA<CovarianceMatricPolicy_>, MotionModelCV<CovarianceMatricPolicy_>>::convertFrom(
    typename MotionModelCA<CovarianceMatricPolicy_>::StateCov&       dstCov,
    const typename MotionModelCV<CovarianceMatricPolicy_>::StateCov& srcCov)
{
  using DstType = MotionModelCA<CovarianceMatricPolicy_>;
  using SrcType = MotionModelCV<CovarianceMatricPolicy_>;

  if constexpr (CovarianceMatricPolicy_::is_factored)
  {
    using FloatType    = typename CovarianceMatricPolicy_::FloatType;
    constexpr auto one = static_cast<FloatType>(1.0);

    math::SquareMatrix<FloatType, DstType::NUM_STATE_VARIABLES> A;
    A.setZeros();
    A.at_unsafe(DstType::X, SrcType::X)   = one;
    A.at_unsafe(DstType::VX, SrcType::VX) = one;
    A.at_unsafe(DstType::Y, SrcType::Y)   = one;
    A.at_unsafe(DstType::VY, SrcType::VY) = one;

    dstCov.setIdentity();
    // copy CV into CA
    dstCov.template fill<SrcType::NUM_STATE_VARIABLES, SrcType::NUM_STATE_VARIABLES>(srcCov);
    // remap indeces by applying the permutation
    dstCov.apaT(A);
    // set ax,ay to variance 1.0
    dstCov.setDiagonal(DstType::AX, one);
    dstCov.setDiagonal(DstType::AY, one);
  }
  else
  {
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
                             SrcType::StateCov::IsRowMajor,
                             DstType::X,
                             DstType::X>(srcCov);
    // copy cross correlations between x,vx and y,vy
    dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                             SrcType::NUM_STATE_VARIABLES,
                             2,
                             2,
                             SrcType::X,
                             SrcType::Y,
                             SrcType::StateCov::IsRowMajor,
                             DstType::X,
                             DstType::Y>(srcCov);
    dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                             SrcType::NUM_STATE_VARIABLES,
                             2,
                             2,
                             SrcType::Y,
                             SrcType::X,
                             SrcType::StateCov::IsRowMajor,
                             DstType::Y,
                             DstType::X>(srcCov);
    // copy y,vy and its correlations
    dstCov.template setBlock<SrcType::NUM_STATE_VARIABLES,
                             SrcType::NUM_STATE_VARIABLES,
                             2,
                             2,
                             SrcType::Y,
                             SrcType::Y,
                             SrcType::StateCov::IsRowMajor,
                             DstType::Y,
                             DstType::Y>(srcCov);
  }
}

} // namespace motion
} // namespace tracking

#endif // BA54EBB4_2356_43F9_BB94_0BC11475E511
