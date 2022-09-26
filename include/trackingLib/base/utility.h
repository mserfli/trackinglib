#ifndef DC13E4C0_4F15_474F_A547_E9BCAFF3909E
#define DC13E4C0_4F15_474F_A547_E9BCAFF3909E

#include <memory>

template <typename T, typename... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T>
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

#endif // DC13E4C0_4F15_474F_A547_E9BCAFF3909E
