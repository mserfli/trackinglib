#ifndef TRACKINGLIB_EXAMPLES_CHECKED_VALUE_H
#define TRACKINGLIB_EXAMPLES_CHECKED_VALUE_H

#include "trackingLib/base/first_include.h" // IWYU pragma: keep

#include <cstdlib>
#include <iostream>
#include <utility>

/// \brief Unwrap a checked result, printing its error and exiting on failure.
/// \tparam T Value type returned by the factory.
/// \tparam E Error enum type returned by the factory.
/// \param[in] result Factory result to check and unwrap.
/// \return The successfully constructed value.
template <typename T, typename E>
auto checkedValue(tl::expected<T, E> result) -> T
{
  if (!result.has_value())
  {
    std::cerr << "Construction failed: " << static_cast<int>(result.error()) << '\n';
    std::exit(EXIT_FAILURE);
  }

  return std::move(result).value();
}

#endif // TRACKINGLIB_EXAMPLES_CHECKED_VALUE_H
