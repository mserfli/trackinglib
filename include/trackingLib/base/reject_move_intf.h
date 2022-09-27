#ifndef E66A1822_C9F9_4EA4_8A35_51620F279CC2
#define E66A1822_C9F9_4EA4_8A35_51620F279CC2

#include <type_traits>

namespace tracking
{
namespace base
{
namespace contract
{

template <typename ImplType>
class RejectMoveIntf
{
public:
  RejectMoveIntf()
  {
    static_assert(!std::is_move_constructible<ImplType>::value, "move ctor not deleted");
    static_assert(!std::is_move_assignable<ImplType>::value, "move assignment operator not deleted");
  }
};

} // namespace contract
} // namespace base
} // namespace tracking

#endif // E66A1822_C9F9_4EA4_8A35_51620F279CC2
