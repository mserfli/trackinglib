#ifndef B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
#define B84C34EE_5B17_49CF_8DC1_3BCF45A59A20

#include "math/linalg/rank1_update.h"

#include "math/analysis/functions.h"
#include "math/linalg/diagonal_matrix.hpp"   // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"            // IWYU pragma: keep
#include <cmath>
#include <limits>

namespace tracking
{
namespace math
{

template <typename FloatType_, sint32 Size_, bool IsRowMajor_>
inline void Rank1Update<FloatType_, Size_, IsRowMajor_>::run(TriangularMatrix<FloatType_, Size_, false, IsRowMajor_>& u,
                                                             DiagonalMatrix<FloatType_, Size_>&                       d,
                                                             FloatType_                                               c,
                                                             Vector<FloatType_, Size_>                                x)
{
  // based on
  // https://scicomp.stackexchange.com/questions/8323/computational-complexity-and-implementation-of-udu-modified-cholesky-rank-1-upda

  if (c == 0)
    return; // no update required

  // update or downdate ?
  const FloatType_ sign = (c > 0) ? +1 : -1;
  c                     = c * sign; // remove the sign
  for (int j = Size_ - 1; j >= 0; --j)
  {
    // Retrieve diagonal and update vector value
    FloatType_ djj = d.at_unsafe(j);
    FloatType_ ujx = x.at_unsafe(j);

    // Compute updated diagonal element with clipping to ensure PSD
    FloatType_ gamma = djj + sign * c * pow<2>(ujx);
    gamma            = std::max(gamma, std::numeric_limits<FloatType_>::epsilon());

    // Update the diagonal
    d.at_unsafe(j) = gamma;

    // Compute scaling factors for U update
    FloatType_ beta = c / gamma;
    FloatType_ eta  = beta * ujx;

    // Update the upper triangular matrix
    for (int i = 0; i < j; ++i)
    {
      FloatType_ uij = u.at_unsafe(i, j);
      x.at_unsafe(i) -= uij * ujx;                      // Update x for future iterations
      u.at_unsafe(i, j) += sign * eta * x.at_unsafe(i); // Apply correction to U
    }

    // Update scaling factor for the next iteration
    c = beta * djj;
  }
}


template <typename FloatType_, sint32 Size_, bool IsRowMajor_>
inline void Rank1Update<FloatType_, Size_, IsRowMajor_>::run(TriangularMatrix<FloatType_, Size_, true, IsRowMajor_>& l,
                                                             DiagonalMatrix<FloatType_, Size_>&                      d,
                                                             FloatType_                                              c,
                                                             Vector<FloatType_, Size_>                               x)
{
  // Methods for Modifying Matrix Factorizations in Mathematics of Computation
  // Gill, Golub, Murray and Saunders (1974)
  //
  // http://stanford.edu/group/SOL/papers/ggms74.pdf

  x *= sqrt(abs(c));
  c = (c > 0) ? static_cast<FloatType_>(1.0) : -static_cast<FloatType_>(1.0);

  FloatType_ dj_{};
  FloatType_ c_{};
  FloatType_ beta{};

  if (c > 0)
  {
    FloatType_ p{};
    c_ = static_cast<FloatType_>(1.0);
    for (auto j = 0; j < Size_; ++j)
    {
      p              = x.at_unsafe(j);
      dj_            = d.at_unsafe(j);
      c              = c_ + pow<2>(p) / dj_;
      d.at_unsafe(j) = dj_ * c / c_;
      beta           = p / (dj_ * c);
      for (auto r = j + 1; r < Size_; ++r)
      {
        x.at_unsafe(r) -= p * l.at_unsafe(r, j);
        l.at_unsafe(r, j) += beta * x.at_unsafe(r);
      }
      c_ = c;
    }
  }
  else
  {
    const auto  l_{l};
    decltype(x) p    = Vector<FloatType_, Size_>{l.solve(x)};
    const auto  dinv = static_cast<const DiagonalMatrix<FloatType_, Size_>&>(d).inverse();
    c_               = std::max(1 - (p.transpose() * (dinv * p)).at_unsafe(0, 0), std::numeric_limits<FloatType_>::epsilon());
    for (auto j = Size_ - 1; j >= 0; --j)
    {
      dj_ = d.at_unsafe(j);
      c   = c_ + pow<2>(p.at_unsafe(j)) / dj_;

      // update d, but ensure PSD
      d.at_unsafe(j) = dj_ * c_ / c;
      d.at_unsafe(j) = std::max(d.at_unsafe(j), std::numeric_limits<FloatType_>::epsilon());

      beta           = -p.at_unsafe(j) / (dj_ * c_);
      x.at_unsafe(j) = p.at_unsafe(j);
      for (auto r = j + 1; r < Size_; ++r)
      {
        l.at_unsafe(r, j) += beta * x.at_unsafe(r);
        x.at_unsafe(r) += p.at_unsafe(j) * l_.at_unsafe(r, j);
      }
      c_ = c;
    }
  }
}

} // namespace math
} // namespace tracking

#endif // B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
