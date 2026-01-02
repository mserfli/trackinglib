#ifndef B445624A_A184_4F7B_A76B_C2CEF55E0C13
#define B445624A_A184_4F7B_A76B_C2CEF55E0C13

#include <type_traits>

namespace tracking
{
namespace base
{
namespace contract
{

#define ERR_MSG_MISSING_FUNCTION "missing function"
#define ERR_MSG_DEFINED_UNEXPECTED_FUNCTION "found unexpected function definition"

// Curiously_recurring_template_pattern
// https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern

// ###############################################################################################################################
//
// Johannes Schaub
// since C++0x
// https://stackoverflow.com/questions/257288/templated-check-for-the-existence-of-a-class-member-function
//
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

// --- an example ---
// template <typename ImplementationType>
// struct Interface
// {
//   Interface()
//   {
//     static_assert(has_bar<ImplementationType, void (ImplementationType::*)(int) const>::value,
//                   "interface not correctly implemented");

//     static_assert(!has_bar<ImplementationType, void (ImplementationType::*)(int)>::value, "interface not correctly
//     implemented");
//   }
//   DECLARE_HAS_MEM_FUNC(bar, has_bar);
// };

// struct test: public Interface<test>
// {
//   void bar(const int) const {}
// };


// ###############################################################################################################################
//
//  Brett Rossier
//  https://stackoverflow.com/questions/87372/check-if-a-class-has-a-member-function-of-a-given-signature
//
//  - Multiple inheritance forces ambiguity of member names.
//  - SFINAE is used to make aliases to member names.
//  - Expression SFINAE is used in just one generic has_member that can accept
//    any alias we pass it.
//

// Variadic to force ambiguity of class members.  C++11 and up.
template <typename... Args>
struct ambiguate: public Args...
{
};

// Non-variadic version of the line above.
// template <typename A, typename B> struct ambiguate : public A, public B {};

template <typename A, typename = void>
struct got_type: std::false_type
{
};

template <typename A>
struct got_type<A>: std::true_type
{
  typedef A type;
};

template <typename T, T>
struct sig_check: std::true_type
{
};

template <typename Alias, typename AmbiguitySeed>
struct has_member
{
  template <typename C>
  static char (&f(decltype(&C::value)))[1];
  template <typename C>
  static char (&f(...))[2];

  // Make sure the member name is consistently spelled the same.
  static_assert((sizeof(f<AmbiguitySeed>(0)) == 1),
                "Member name specified in AmbiguitySeed is different from member name specified in Alias, or wrong "
                "Alias/AmbiguitySeed has been specified.");

  static bool const value = sizeof(f<Alias>(0)) == 2;
};


// Check for any member with given name, whether var, func, class, union, enum.
#define CREATE_MEMBER_CHECK(member)                                                                                              \
                                                                                                                                 \
  template <typename T, typename = std::true_type>                                                                               \
  struct Alias_##member;                                                                                                         \
                                                                                                                                 \
  template <typename T>                                                                                                          \
  struct Alias_##member<T, std::integral_constant<bool, got_type<decltype(&T::member)>::value>>                                  \
  {                                                                                                                              \
    static const decltype(&T::member) value;                                                                                     \
  };                                                                                                                             \
                                                                                                                                 \
  struct AmbiguitySeed_##member                                                                                                  \
  {                                                                                                                              \
    char member;                                                                                                                 \
  };                                                                                                                             \
                                                                                                                                 \
  template <typename T>                                                                                                          \
  struct has_member_##member                                                                                                     \
  {                                                                                                                              \
    static const bool value =                                                                                                    \
        has_member<Alias_##member<ambiguate<T, AmbiguitySeed_##member>>, Alias_##member<AmbiguitySeed_##member>>::value;         \
  }


// Check for member variable with given name.
#define CREATE_MEMBER_VAR_CHECK(var_name)                                                                                        \
                                                                                                                                 \
  template <typename T, typename = std::true_type>                                                                               \
  struct has_member_var_##var_name: std::false_type                                                                              \
  {                                                                                                                              \
  };                                                                                                                             \
                                                                                                                                 \
  template <typename T>                                                                                                          \
  struct has_member_var_##var_name<                                                                                              \
      T,                                                                                                                         \
      std::integral_constant<bool, !std::is_member_function_pointer<decltype(&T::var_name)>::value>>: std::true_type             \
  {                                                                                                                              \
  }


// Check for member function with given name AND signature.
#define CREATE_MEMBER_FUNC_SIG_CHECK(func_name, func_sig, templ_postfix)                                                         \
                                                                                                                                 \
  template <typename T, typename = std::true_type>                                                                               \
  struct has_member_func_##templ_postfix: std::false_type                                                                        \
  {                                                                                                                              \
  };                                                                                                                             \
                                                                                                                                 \
  template <typename T>                                                                                                          \
  struct has_member_func_##templ_postfix<T, std::integral_constant<bool, sig_check<func_sig, &T::func_name>::value>>             \
      : std::true_type                                                                                                           \
  {                                                                                                                              \
  }


// Check for member class with given name.
#define CREATE_MEMBER_CLASS_CHECK(class_name)                                                                                    \
                                                                                                                                 \
  template <typename T, typename = std::true_type>                                                                               \
  struct has_member_class_##class_name: std::false_type                                                                          \
  {                                                                                                                              \
  };                                                                                                                             \
                                                                                                                                 \
  template <typename T>                                                                                                          \
  struct has_member_class_##class_name<                                                                                          \
      T,                                                                                                                         \
      std::integral_constant<bool, std::is_class<typename got_type<typename T::class_name>::type>::value>>: std::true_type       \
  {                                                                                                                              \
  }


// Check for member union with given name.
#define CREATE_MEMBER_UNION_CHECK(union_name)                                                                                    \
                                                                                                                                 \
  template <typename T, typename = std::true_type>                                                                               \
  struct has_member_union_##union_name: std::false_type                                                                          \
  {                                                                                                                              \
  };                                                                                                                             \
                                                                                                                                 \
  template <typename T>                                                                                                          \
  struct has_member_union_##union_name<                                                                                          \
      T,                                                                                                                         \
      std::integral_constant<bool, std::is_union<typename got_type<typename T::union_name>::type>::value>>: std::true_type       \
  {                                                                                                                              \
  }


// Check for member enum with given name.
#define CREATE_MEMBER_ENUM_CHECK(enum_name)                                                                                      \
                                                                                                                                 \
  template <typename T, typename = std::true_type>                                                                               \
  struct has_member_enum_##enum_name: std::false_type                                                                            \
  {                                                                                                                              \
  };                                                                                                                             \
                                                                                                                                 \
  template <typename T>                                                                                                          \
  struct has_member_enum_##enum_name<                                                                                            \
      T,                                                                                                                         \
      std::integral_constant<bool, std::is_enum<typename got_type<typename T::enum_name>::type>::value>>: std::true_type         \
  {                                                                                                                              \
  }


// Check for function with given name, any signature.
#define CREATE_MEMBER_FUNC_CHECK(func)                                                                                           \
  template <typename T>                                                                                                          \
  struct has_member_func_##func                                                                                                  \
  {                                                                                                                              \
    static const bool value = has_member_##func<T>::value && !has_member_var_##func<T>::value &&                                 \
                              !has_member_class_##func<T>::value && !has_member_union_##func<T>::value &&                        \
                              !has_member_enum_##func<T>::value;                                                                 \
  }

#define CREATE_MEMBER_CHECKS(member)                                                                                             \
  CREATE_MEMBER_CHECK(member);                                                                                                   \
  CREATE_MEMBER_VAR_CHECK(member);                                                                                               \
  CREATE_MEMBER_CLASS_CHECK(member);                                                                                             \
  CREATE_MEMBER_UNION_CHECK(member);                                                                                             \
  CREATE_MEMBER_ENUM_CHECK(member);                                                                                              \
  CREATE_MEMBER_FUNC_CHECK(member)

// ###############################################################################################################################
//
// Enlico
// https://stackoverflow.com/questions/87372/check-if-a-class-has-a-member-function-of-a-given-signature
//


// ###############################################################################################################################
//
// Mike Kinghan
// Matthias Serfling: slightly modified to support constness
// https://stackoverflow.com/questions/87372/check-if-a-class-has-a-member-function-of-a-given-signature
//
template <typename T, typename E, bool Const>
struct has_round_brackets_op_int_int
{
  /* SFINAE operator-has-correct-sig :) */
  template <bool Const_, typename A>
  struct SignatureTest
  {
  };

  template <typename A>
  struct SignatureTest<true, A>
  {
    static auto test(E (A::*)(int, int) const) -> std::true_type { return {}; }
  };

  template <typename A>
  struct SignatureTest<false, A>
  {
    static auto test(E& (A::*)(int, int)) -> std::true_type { return {}; }
  };

  /* SFINAE operator-exists :) */
  template <typename A>
  static auto test(decltype(&A::operator()), void*) -> decltype(SignatureTest<Const, A>::test(&A::operator()))
  {
    /* Operator exists. What about sig? */
    using return_type = decltype(SignatureTest<Const, A>::test(&A::operator()));
    return return_type();
  }

  /* SFINAE game over :( */
  template <typename A>
  static auto test(...) -> std::false_type
  {
    return {};
  }

  /* This will be either `std::true_type` or `std::false_type` */
  using type = decltype(test<T>(0, 0));

  static const bool value = type::value; /* Which is it? */
};

template <typename T, typename E, bool Const>
struct has_square_brackets_op_int
{
  /* SFINAE operator-has-correct-sig :) */
  template <bool Const_, typename A>
  struct SignatureTest
  {
  };

  template <typename A>
  struct SignatureTest<true, A>
  {
    static auto test(E (A::*)(int) const) -> std::true_type { return {}; }
  };

  template <typename A>
  struct SignatureTest<false, A>
  {
    static auto test(E& (A::*)(int)) -> std::true_type { return {}; }
  };

  /* SFINAE operator-exists :) */
  template <typename A>
  static auto test(decltype(&A::operator[]), void*) -> decltype(SignatureTest<Const, A>::test(&A::operator[]))
  {
    /* Operator exists. What about sig? */
    using return_type = decltype(SignatureTest<Const, A>::test(&A::operator[]));
    return return_type();
  }

  /* SFINAE game over :( */
  template <typename A>
  static auto test(...) -> std::false_type
  {
    return {};
  }

  /* This will be either `std::true_type` or `std::false_type` */
  using type = decltype(test<T>(0, 0));

  static const bool value = type::value; /* Which is it? */
};

} // namespace contract
} // namespace base
} // namespace tracking

#endif // B445624A_A184_4F7B_A76B_C2CEF55E0C13
