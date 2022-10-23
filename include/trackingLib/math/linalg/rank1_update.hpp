#ifndef B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
#define B84C34EE_5B17_49CF_8DC1_3BCF45A59A20

#include "math/linalg/rank1_update.h"

#include "math/analysis/functions.h"
#include "math/linalg/square_matrix.hpp"
#include "math/linalg/triangular_matrix.hpp"
#include "math/linalg/diagonal_matrix.hpp"
#include <limits>
#include <type_traits>

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
inline void Rank1Update<FloatType, Size>::run(TriangularMatrix<FloatType, Size, false>& u,
                                              DiagonalMatrix<FloatType, Size>&          d,
                                              FloatType                                 c,
                                              Vector<FloatType, Size>                   x)
{
  // Navigation Filter Best Practices
  //   Carpenter, Russell J. and D'Souza, Christopher N.
  //   NASA
  // page 89

  assert(c > 0 && "algorithm is designed for KF time update where P grows and thus c is positive");

  FloatType beta{};
  FloatType eta{};
  FloatType dj{};
  for (auto j = Size - 1; j > 0; --j)
  {
    dj      = d[j];
    d[j]    = std::max(dj + (c * pow<2>(x[j])), std::numeric_limits<FloatType>::epsilon());
    u(j, j) = static_cast<FloatType>(1.0);

    beta = c / d[j]; // division is safe because of std::max before
    eta  = beta * x[j];
    for (auto i = 0; i < j; ++i)
    {
      x[i] -= u(i, j) * x[j];
      u(i, j) += x[i] * eta;
    }
    c = beta * dj; // use here the backup value if d[j] stored at the beginning
  }
  d[0] += c * pow<2>(x[0]);
}

template <typename FloatType, sint32 Size>
inline void Rank1Update<FloatType, Size>::run(TriangularMatrix<FloatType, Size, true>& l,
                                              DiagonalMatrix<FloatType, Size>&         d,
                                              FloatType                                c,
                                              Vector<FloatType, Size>                  x)
{
  // Methods for Modifying Matrix Factorizations in Mathematics of Computation
  // Gill, Golub, Murray and Saunders (1974)
  //
  // http://stanford.edu/group/SOL/papers/ggms74.pdf

  x *= sqrt(abs(c));
  c = (c > 0) ? static_cast<FloatType>(1.0) : -static_cast<FloatType>(1.0);

  FloatType dj_{};
  FloatType c_{};
  FloatType beta{};

  if (c > 0)
  {
    FloatType p{};
    c_ = static_cast<FloatType>(1.0);
    for (auto j = 0; j < Size; ++j)
    {
      p    = x[j];
      dj_  = d[j];
      c    = c_ + pow<2>(p) / dj_;
      d[j] = dj_ * c / c_;
      beta = p / (dj_ * c);
      for (auto r = j + 1; r < Size; ++r)
      {
        x[r] -= p * l(r, j);
        l(r, j) += beta * x[r];
      }
      c_ = c;
    }
  }
  else
  {
    const auto  l_{l};
    decltype(x) p    = l.solve(x);
    const auto  dinv = static_cast<const DiagonalMatrix<FloatType, Size>&>(d).inverse();
    c_               = std::max(1 - (p.transpose() * (dinv * p))(0, 0), std::numeric_limits<FloatType>::epsilon());
    for (auto j = Size - 1; j >= 0; --j)
    {
      dj_  = d[j];
      c    = c_ + pow<2>(p[j]) / dj_;
      d[j] = dj_ * c_ / c;
      beta = -p[j] / (dj_ * c_);
      x[j] = p[j];
      for (auto r = j + 1; r < Size; ++r)
      {
        l(r, j) += beta * x[r];
        x[r] += p[j] * l_(r, j);
      }
      c_ = c;
    }
  }
}

} // namespace math
} // namespace tracking

#endif // B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
