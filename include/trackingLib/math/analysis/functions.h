#ifndef B8E086B0_F27F_43B8_9748_96B243918772
#define B8E086B0_F27F_43B8_9748_96B243918772

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace detail
{
template <class T, int N>
struct helper
{
  static constexpr T pow(const T x) { return helper<T, N - 1>::pow(x) * x; }
};

template <class T>
struct helper<T, 1>
{
  static constexpr T pow(const T x) { return x; }
};

template <class T>
struct helper<T, 0>
{
  static constexpr T pow(const T) { return 1; }
};
} // namespace detail

template <int N, class T>
T constexpr pow(T const x)
{
  return detail::helper<T, N>::pow(x);
}

} // namespace math
} // namespace tracking

#endif // B8E086B0_F27F_43B8_9748_96B243918772
