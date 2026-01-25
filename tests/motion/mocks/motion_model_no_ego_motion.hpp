#ifndef BE754B97_D814_4E55_A9C2_6A546BC5361A
#define BE754B97_D814_4E55_A9C2_6A546BC5361A

#include "gmock/gmock.h"
#include "trackingLib/env/ego_motion.h"

namespace test
{
template <typename MM>
class MotionModelNoEgoMotionMock: public MM
{
public:
  using MM::MM;
  using EgoMotionType = typename MM::EgoMotionType;
  MOCK_METHOD(void,
              compensateEgoMotion,
              (typename MM::EgoMotionMappingMatrix&, typename MM::StateMatrix&, const EgoMotionType&),
              (override));
  void mock_compensateEgoMotion(typename MM::EgoMotionMappingMatrix&, typename MM::StateMatrix& Go, const EgoMotionType&)
  {
    Go.setIdentity();
  }
  void delegate()
  {
    ON_CALL(*this, compensateEgoMotion)
        .WillByDefault([this](typename MM::EgoMotionMappingMatrix& Ge,
                              typename MM::StateMatrix&            Go,
                              const EgoMotionType&                 egoMotion) { mock_compensateEgoMotion(Ge, Go, egoMotion); });
  }
};
} // namespace test

#endif // BE754B97_D814_4E55_A9C2_6A546BC5361A
