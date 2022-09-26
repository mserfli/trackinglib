#ifndef B445624A_A184_4F7B_A76B_C2CEF55E0C13
#define B445624A_A184_4F7B_A76B_C2CEF55E0C13

#include <type_traits>

// Curiously_recurring_template_pattern 
// https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern

// since C++0x
// https://stackoverflow.com/questions/257288/templated-check-for-the-existence-of-a-class-member-function

// since C++11, but no viable solution to check for member function constness
// https://stackoverflow.com/questions/44317289/c-interface-without-virtual-functions

#define DECLARE_HAS_MEM_FUNC(func, name)                                                                                         \
  template <typename _T, typename _Signature>                                                                                    \
  struct name                                                                                                                    \
  {                                                                                                                              \
    typedef short yes;                                                                                                           \
    typedef int   no;                                                                                                            \
    template <typename _U, _U>                                                                                                   \
    struct type_check;                                                                                                           \
    template <typename _1>                                                                                                       \
    static yes& chk(type_check<_Signature, &_1::func>*); /* NOLINT(modernize-use-trailing-return-type) */                        \
    template <typename>                                                                                                          \
    static no&        chk(...); /* NOLINT(modernize-use-trailing-return-type) */                                                 \
    static bool const value = sizeof(chk<_T>(0)) == sizeof(yes);                                                                 \
  }

// --- an example ----------------------------------------------------------------------------------------------------------------
// template <typename ImplementationType>
// struct Interface
// {
//   Interface()
//   {
//     static_assert(has_bar<ImplementationType, void (ImplementationType::*)(int) const>::value,
//                   "interface not correctly implemented");

//     static_assert(!has_bar<ImplementationType, void (ImplementationType::*)(int)>::value, "interface not correctly implemented");
//   }
//   DECLARE_HAS_MEM_FUNC(bar, has_bar);
// };

// struct test: public Interface<test>
// {
//   void bar(const int) const {}
// };

#endif // B445624A_A184_4F7B_A76B_C2CEF55E0C13
