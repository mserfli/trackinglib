#ifndef B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
#define B84C34EE_5B17_49CF_8DC1_3BCF45A59A20

#include "math/linalg/rank1_update.h"

#include "math/analysis/functions.h"
#include "math/linalg/diagonal_matrix.hpp"   // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"            // IWYU pragma: keep
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
  // Navigation Filter Best Practices
  //   Carpenter, Russell J. and D'Souza, Christopher N.
  //   NASA
  // page 89

  assert(c > 0 && "algorithm is designed for KF time update where P grows and thus c is positive");

  FloatType_ beta{};
  FloatType_ eta{};
  FloatType_ dj{};
  for (auto j = Size_ - 1; j > 0; --j)
  {
    dj                = d.at_unsafe(j);
    d.at_unsafe(j)    = std::max(dj + (c * pow<2>(x.at_unsafe(j))), std::numeric_limits<FloatType_>::epsilon());
    u.at_unsafe(j, j) = static_cast<FloatType_>(1.0);

    beta = c / d.at_unsafe(j); // division is safe because of std::max before
    eta  = beta * x.at_unsafe(j);
    for (auto i = 0; i < j; ++i)
    {
      x.at_unsafe(i) -= u.at_unsafe(i, j) * x.at_unsafe(j);
      u.at_unsafe(i, j) += x.at_unsafe(i) * eta;
    }
    c = beta * dj; // use here the backup value if d[j] stored at the beginning
  }
  d.at_unsafe(0) += c * pow<2>(x.at_unsafe(0));
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
      dj_            = d.at_unsafe(j);
      c              = c_ + pow<2>(p.at_unsafe(j)) / dj_;
      d.at_unsafe(j) = dj_ * c_ / c;
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
